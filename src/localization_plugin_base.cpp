#include "ed_localization/localization_plugin_base.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geolib/Shape.h>
#include <geolib/Box.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf2_conversions.h>

#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/MarkerArray.h>

#include <ed/update_request.h>

namespace ed_localization {

// ----------------------------------------------------------------------------------------------------

LocalizationPluginBase::LocalizationPluginBase() :
    resample_interval_(1),
    resample_count_(0),
    update_min_d_sq_(0),
    update_min_a_(0),
    have_previous_odom_pose_(false),
    latest_map_odom_valid_(false),
    last_map_size_revision_(0),
    tf_buffer_(),
    tf_listener_(nullptr),
    tf_broadcaster_(nullptr),
    old_msg_size_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPluginBase::~LocalizationPluginBase()
{
    // Get transform between map and odom frame
    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_.lookupTransform(map_frame_id_, odom_frame_id_, ros::Time(0));
        tf2::Quaternion rotation_map_odom;
        tf2::convert(ts.transform.rotation, rotation_map_odom); // Returns a quaternion

        // Store the x, y and yaw on the parameter server
        ros::NodeHandle nh;
        nh.setParam("initial_pose/x", ts.transform.translation.x);
        nh.setParam("initial_pose/y", ts.transform.translation.y);
        nh.setParam("initial_pose/yaw", rotation_map_odom.getAngle());
    }
    catch (tf2::TransformException ex)
    {
        ROS_ERROR_STREAM_NAMED("localization", ex.what());
    }

    cv::destroyAllWindows();
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::configure(tue::Configuration config)
{
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    config.value("resample_interval", resample_interval_);

    double update_min_d;
    config.value("update_min_d", update_min_d);
    update_min_d_sq_ = update_min_d * update_min_d;
    config.value("update_min_a", update_min_a_);

    if (config.readGroup("odom_model", tue::config::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        odom_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("particle_filter", tue::config::REQUIRED))
    {
        particle_filter_.configure(config);
        config.endGroup();
    }

    double tmp_transform_tolerance = 0.1;
    config.value("transform_tolerance", tmp_transform_tolerance, tue::config::OPTIONAL);
    transform_tolerance_.fromSec(tmp_transform_tolerance);

    if (config.hasError())
        return;

    if (!configureImpl(config))
        return;

    ros::NodeHandle nh;

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationPluginBase::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    geo::Transform2d initial_pose = getInitialPose(nh, config);

    initParticleFilterUniform(initial_pose);

    config.value("robot_name", robot_name_);

    pub_particles_ = nh.advertise<visualization_msgs::MarkerArray>("ed/localization/particles", 10);
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationPluginBase::configureImpl(tue::Configuration /*config*/)
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2d LocalizationPluginBase::getInitialPose(const ros::NodeHandle& nh, tue::Configuration& config)
{
    try
    {
        return tryGetInitialPoseFromParamServer(nh);
    }
    catch (const ConfigurationException&)
    {
    }

    try
    {
        return tryGetInitialPoseFromConfig(config);
    }
    catch (const ConfigurationException&)
    {
    }

    return geo::Transform2d::identity();
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPluginBase::tryGetInitialPoseFromParamServer(const ros::NodeHandle& nh)
{
    // Getting last pose from parameter server
    std::map<std::string, double> ros_param_position;
    if (!nh.getParam("initial_pose", ros_param_position))
    {
        std::string msg = "Could not read initial pose from the parameter server";
        ROS_WARN_STREAM_NAMED("localization", msg);
        throw ConfigurationException(msg);
    }

    // Make a homogeneous transformation with the variables from the parameter server
    tf2::Transform homogtrans_map_odom;
    homogtrans_map_odom.setOrigin(tf2::Vector3(ros_param_position["x"], ros_param_position["y"], 0.0));
    tf2::Quaternion q_map_odom;
    q_map_odom.setRPY(0, 0, ros_param_position["yaw"]);
    homogtrans_map_odom.setRotation(q_map_odom);

    if (!tf_buffer_.canTransform(odom_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(1)))
    {
        std::string msg = "No transform between odom and base_link";
        ROS_ERROR_STREAM_NAMED("localization", msg);
        throw ConfigurationException(msg);
    }

    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_.lookupTransform(odom_frame_id_, base_link_frame_id_, ros::Time(0));
        tf2::Stamped<tf2::Transform> tf_odom_base_link;
        tf2::convert(ts, tf_odom_base_link);
        // Calculate base link position in map frame
        tf2::Transform tf_map_base_link = homogtrans_map_odom * tf_odom_base_link;
        tf2::Vector3 pos_map_baselink = tf_map_base_link.getOrigin(); // Returns a vector
        tf2::Quaternion rotation_map_baselink = tf_map_base_link.getRotation(); // Returns a quaternion

        geo::Transform2d result;
        result.t.x = pos_map_baselink.x();
        result.t.y = pos_map_baselink.y();
        result.setRotation(rotation_map_baselink.getAngle());

        ROS_DEBUG_STREAM_NAMED("localization", "Initial pose from parameter server: [" <<
            result.t.x << ", " << result.t.y << "], yaw:" << result.rotation()
        );
        return result;
    }
    catch (const tf2::TransformException& ex)
    {
        std::string msg = std::string(ex.what());
        ROS_ERROR_STREAM_NAMED("localization", msg);
        throw ConfigurationException(msg);
    }
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPluginBase::tryGetInitialPoseFromConfig(tue::Configuration& config)
{
    double x, y, yaw;
    if (config.readGroup("initial_pose", tue::config::OPTIONAL))
    {
        config.value("x", x);
        config.value("y", y);
        config.value("rz", yaw);
        config.endGroup();
    }
    else
    {
        std::string message = "Initial pose not present in config";
        ROS_WARN_STREAM_NAMED("localization", message);
        throw ConfigurationException(message);
    }

    geo::Transform2d result(x, y, yaw); // NOLINT(clang-analyzer-core.CallAndMessage)
    return result;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    initial_pose_msg_.reset();
    cb_queue_.callAvailable();

    if (initial_pose_msg_)
    {
        // Set initial pose
        geo::Pose3D pose;
        geo::convert(initial_pose_msg_->pose.pose, pose);
        initParticleFilterUniform(pose.projectTo2d());
    }

    processImpl(world, req);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::processImpl(const ed::WorldModel& /*world*/, ed::UpdateRequest& /*req*/)
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::initParticleFilterUniform(const geo::Transform2& pose)
{
    const geo::Vec2& p = pose.getOrigin();
    const double yaw = pose.rotation();
    particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), yaw - 0.15, yaw + 0.15);
    have_previous_odom_pose_ = false;
    resample_count_ = 0;
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationPluginBase::resample(const ed::WorldModel& world)
{
    if(++resample_count_ % resample_interval_)
        return false;

    ROS_DEBUG_NAMED("localization", "resample particle filter");
    const std::function<void()> update_map_size_func = std::bind(&LocalizationPluginBase::updateMapSize, this, std::ref(world));
    const std::function<geo::Transform2()> gen_random_pose_func = std::bind(&LocalizationPluginBase::generateRandomPose, this, update_map_size_func);
    particle_filter_.resample(gen_random_pose_func);
    return true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::publishParticles(const ros::Time &stamp)
{
    ROS_DEBUG_NAMED("localization", "Publishing particles");
    const std::vector<Sample>& samples = particle_filter_.samples();
    visualization_msgs::MarkerArray particles_msg;
    particles_msg.markers.resize(std::max<uint>(old_msg_size_, samples.size()));
    for(uint i = 0; i < samples.size(); ++i)
    {
        visualization_msgs::Marker& marker = particles_msg.markers[i];
        const Sample& sample = samples[i];
        marker.header.frame_id = map_frame_id_;
        marker.header.stamp = stamp;

        const geo::Transform2& p = sample.pose;
        geo::convert(p.projectTo3d(), marker.pose);

        marker.type = marker.ARROW;
        marker.ns = std::to_string(i);
        marker.action = marker.ADD;
        marker.scale.x = 0.5;
        marker.scale.z = marker.scale.y = 0.05*exp(sample.weight);
        marker.color.r = 1;
        marker.color.a = 1;
    }
    for (uint i = samples.size(); i<old_msg_size_; ++i)
    {
        visualization_msgs::Marker& marker = particles_msg.markers[i];
        marker.ns = std::to_string(i);
        marker.action = marker.DELETE;
    }
    old_msg_size_ = samples.size();

    pub_particles_.publish(particles_msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::updateMapOdom(const geo::Pose3D& odom_to_base_link)
{
    ROS_DEBUG_NAMED("localization", "Updating map_odom");
    // Get the best pose (2D)
    geo::Transform2 mean_pose = particle_filter_.calculateMeanPose();
    ROS_DEBUG_STREAM("mean_pose: x: " << mean_pose.t.x << ", y: " << mean_pose.t.y << ", yaw: " << mean_pose.rotation());

    // Convert best pose to 3D
    geo::Pose3D map_to_base_link;
    map_to_base_link = mean_pose.projectTo3d();

    latest_map_odom_ = map_to_base_link * odom_to_base_link.inverse();
    latest_map_odom_valid_ = true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::publishMapOdom(const ros::Time &stamp)
{
    ROS_DEBUG_STREAM_NAMED("localization", "Publishing map_odom with timestamp(incl. tolerance: " << transform_tolerance_ << "): " << stamp + transform_tolerance_);
    // Convert to TF transform
    geometry_msgs::TransformStamped latest_map_odom_tf;
    geo::convert(latest_map_odom_, latest_map_odom_tf.transform);

    // Set frame id's and time stamp
    latest_map_odom_tf.header.frame_id = map_frame_id_;
    latest_map_odom_tf.child_frame_id = odom_frame_id_;
    latest_map_odom_tf.header.stamp = stamp + transform_tolerance_;

    // Publish TF
    tf_broadcaster_->sendTransform(latest_map_odom_tf);
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPluginBase::transform(const std::string& target_frame, const std::string& source_frame,
                                              const ros::Time& time, geometry_msgs::TransformStamped& transform)
{
    try
    {
        transform = tf_buffer_.lookupTransform(target_frame, source_frame, time);
        return OK;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error
            // (i.e., the scan is too old or too new, respectively)
            transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));

            if (time > transform.header.stamp)
            {
                // Scan is too new
                return TOO_RECENT;
            }
            else
            {
                // Otherwise it has to be too old
                return TOO_OLD;
            }
        }
        catch (const tf2::TransformException& exc)
        {
            return UNKNOWN_ERROR;
        }
    }
    catch (const tf2::TransformException& ex)
    {
        return UNKNOWN_ERROR;
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    initial_pose_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPluginBase::generateRandomPose(std::function<void()> update_map_size)
{
    update_map_size();
    geo::Transform2 pose;
    pose.t = min_map_;
    const geo::Vec2 map_size = max_map_ - min_map_;
    pose.t.x +=  drand48() * map_size.x;
    pose.t.y +=  drand48() * map_size.y;
    pose.setRotation(drand48() * 2 * M_PI - M_PI);
    return pose;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPluginBase::updateMapSize(const ed::WorldModel& world)
{
    if (world.revision() <= last_map_size_revision_)
        return;

    geo::Vec2 min(1e6, 1e6), max(-1e6, -1e6);

    for (auto it = world.begin(); it != world.end(); ++ it)
    {
        const ed::EntityConstPtr& e = *it;

        const geo::ShapeConstPtr& shape = e->shape();

        // Skip robot and entities without pose or shape
        if (!e->has_pose() || !shape || e->hasFlag("self") || shape->getBoundingBox().getMax().z < 0.05)
            continue;

        geo::Vector3 min_entity_world = e->pose() * shape->getBoundingBox().getMin();
        geo::Vector3 max_entity_world = e->pose() * shape->getBoundingBox().getMax();

        min.x = std::min<double>(min.x, min_entity_world.x);
        min.y = std::min<double>(min.y, min_entity_world.y);
        max.x = std::max<double>(max.x, max_entity_world.x);
        max.y = std::max<double>(max.y, max_entity_world.y);
    }

    min_map_ = min;
    max_map_ = max;
    last_map_size_revision_ = world.revision();

}

// ----------------------------------------------------------------------------------------------------

} // END NS ed_localization
