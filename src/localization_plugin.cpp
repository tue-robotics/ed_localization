#include <exception>
#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tue/profiling/timer.h>

#include <geolib/Shape.h>
#include <geolib/Box.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf2_conversions.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <ed/update_request.h>

class ConfigurationException: public std::exception
{
public:
    ConfigurationException(const std::string& msg){message_ = msg;}

    virtual const char* what() const throw()
    {
        return message_.c_str();
    }
private:
    std::string message_;
};

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() :
    visualize_(false),
    resample_interval_(1),
    resample_count_(0),
    update_min_d_(0),
    update_min_a_(0),
    have_previous_odom_pose_(false),
    latest_map_odom_valid_(false),
    update_(false),
    laser_offset_initialized_(false),
    last_map_size_revision_(0),
    tf_broadcaster_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{

    // Get transform between map and odom frame
    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_->lookupTransform(map_frame_id_, odom_frame_id_, ros::Time(0));
        tf2::Quaternion rotation_map_odom;
        tf2::convert(ts.transform.rotation, rotation_map_odom); // Returns a quaternion

        // Store the x, y and yaw on the parameter server
        ros::NodeHandle nh;
        nh.setParam("initial_pose/x", ts.transform.translation.x);
        nh.setParam("initial_pose/y", ts.transform.translation.y);
        nh.setParam("initial_pose/yaw", rotation_map_odom.getAngle());
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_ERROR_STREAM_NAMED("Localization", ex.what());
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::configure(tue::Configuration config)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    visualize_ = false;
    config.value("visualize", visualize_, tue::config::OPTIONAL);

    config.value("resample_interval", resample_interval_);

    config.value("update_min_d", update_min_d_);
    config.value("update_min_a", update_min_a_);

    std::string laser_topic;

    if (config.readGroup("odom_model", tue::config::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        odom_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("laser_model", tue::config::REQUIRED))
    {
        config.value("topic", laser_topic);
        laser_model_.configure(config);
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

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationPlugin::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    geo::Transform2d initial_pose = getInitialPose(nh, config);

    initParticleFilterUniform(initial_pose);

    config.value("robot_name", robot_name_);

    pub_particles_ = nh.advertise<geometry_msgs::PoseArray>("ed/localization/particles", 10);
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2d LocalizationPlugin::getInitialPose(const ros::NodeHandle& nh, tue::Configuration& config)
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

geo::Transform2 LocalizationPlugin::tryGetInitialPoseFromParamServer(const ros::NodeHandle& nh)
{
    // Getting last pose from parameter server
    std::map<std::string, double> ros_param_position;
    if (!nh.getParam("initial_pose", ros_param_position))
    {
        std::string msg = "Could not read initial pose from the parameter server";
        ROS_WARN_STREAM_NAMED("Localization", msg);
        throw ConfigurationException(msg);
    }

    // Make a homogeneous transformation with the variables from the parameter server
    tf2::Transform homogtrans_map_odom;
    homogtrans_map_odom.setOrigin(tf2::Vector3(ros_param_position["x"], ros_param_position["y"], 0.0));
    tf2::Quaternion q_map_odom;
    q_map_odom.setRPY(0, 0, ros_param_position["yaw"]);
    homogtrans_map_odom.setRotation(q_map_odom);

    if (!tf_buffer_->canTransform(odom_frame_id_, base_link_frame_id_, ros::Time(0), ros::Duration(1)))
    {
        std::string msg = "No transform between odom and base_link";
        ROS_ERROR_STREAM_NAMED("Localization", msg);
        throw ConfigurationException(msg);
    }

    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_->lookupTransform(odom_frame_id_, base_link_frame_id_, ros::Time(0));
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

        ROS_DEBUG_STREAM_NAMED("Localization", "Initial pose from parameter server: [" <<
            result.t.x << ", " << result.t.y << "], yaw:" << result.rotation()
        );
        return result;
    }
    catch (const tf2::TransformException& ex)
    {
        std::string msg = std::string(ex.what());
        ROS_ERROR_STREAM_NAMED("Localization", msg);
        throw ConfigurationException(msg);
    }
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::tryGetInitialPoseFromConfig(tue::Configuration& config)
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
        ROS_WARN_STREAM_NAMED("Localization", message);
        throw ConfigurationException(message);
    }

    geo::Transform2d result(x, y, yaw);
    return result;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
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

    while(!scan_buffer_.empty())
    {
        TransformStatus status = update(scan_buffer_.front(), world, req);
        if (status == OK || status == TOO_OLD || status == UNKNOWN_ERROR)
            scan_buffer_.pop();
        else
            break;
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::update(const sensor_msgs::LaserScanConstPtr& scan, const ed::WorldModel& world, ed::UpdateRequest& req)
{
    //  Get transformation from base_link to laser_frame
    if (!laser_offset_initialized_)
    {
        TransformStatus ts = initLaserOffset(scan->header.frame_id, scan->header.stamp);
        if (ts != OK)
            return ts;
    }


    // Check if particle filter is initialized
    if (particle_filter_.samples().empty())
    {
        ROS_ERROR_NAMED("Localization", "(update) Empty particle filter");
        return UNKNOWN_ERROR;
    }


    // Calculate delta movement based on odom (fetched from TF)
    geo::Pose3D odom_to_base_link;
    geo::Transform2 movement;

    tf2::Stamped<tf2::Transform> odom_to_base_link_tf;
    TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, scan->header.stamp, odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf, odom_to_base_link);

    if (have_previous_odom_pose_)
    {
        // Get displacement and project to 2D
        movement = (previous_odom_pose_.inverse() * odom_to_base_link).projectTo2d();

        update_ = std::abs(movement.t.x) >= update_min_d_ || std::abs(movement.t.y) >= update_min_d_ || std::abs(movement.rotation()) >= update_min_a_;
    }

    bool force_publication = false;
    if (!have_previous_odom_pose_)
    {
        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;
        update_ = true;
        force_publication = true;
    }
    else if (have_previous_odom_pose_ && update_)
    {
        // Update motion
        odom_model_.updatePoses(movement, particle_filter_);
    }

    bool resampled = false;
    if (update_)
    {
        ROS_DEBUG_NAMED("Localization", "Updating laser");
        // Update sensor
        laser_model_.updateWeights(world, *scan, particle_filter_);

        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;

        update_ = false;


        // (Re)sample
        resampled = resample(world);

        // Publish particles
        publishParticles(scan->header.stamp);
    }

    // Update map-odom
    if(resampled || force_publication)
    {
        updateMapOdom(odom_to_base_link);
    }

    // Publish result
    if (latest_map_odom_valid_)
    {
        publishMapOdom(scan->header.stamp);

        // This should be executed allways. map_odom * odom_base_link
        if (!robot_name_.empty())
            req.setPose(robot_name_, latest_map_odom_ * previous_odom_pose_);
    }

    // Visualization
    if (visualize_)
    {
        visualize();
    }
    else
    {
        cv::destroyAllWindows();
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::initLaserOffset(const std::string& frame_id, const ros::Time& stamp)
{
    tf2::Stamped<tf2::Transform> p_laser;
    TransformStatus ts = transform(base_link_frame_id_, frame_id, stamp, p_laser);

    if (ts != OK)
        return ts;

    geo::Transform2 offset(geo::Mat2(p_laser.getBasis()[0][0], p_laser.getBasis()[0][1],
                                     p_laser.getBasis()[1][0], p_laser.getBasis()[1][1]),
                           geo::Vec2(p_laser.getOrigin().getX(), p_laser.getOrigin().getY()));

    bool upside_down = p_laser.getBasis()[2][2] < 0;
    if (upside_down)
    {
        offset.R.yx = -offset.R.yx;
        offset.R.yy = -offset.R.yy;
    }

    double laser_height = p_laser.getOrigin().getZ();

    laser_model_.setLaserOffset(offset, laser_height, upside_down);

    laser_offset_initialized_ = true;

    return OK;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initParticleFilterUniform(const geo::Transform2& pose)
{
    const geo::Vec2& p = pose.getOrigin();
    const double yaw = pose.rotation();
    particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), yaw - 0.3, yaw + 0.3);
    have_previous_odom_pose_ = false;
    resample_count_ = 0;
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationPlugin::resample(const ed::WorldModel& world)
{
    if(++resample_count_ % resample_interval_)
        return false;

    ROS_DEBUG_NAMED("Localization", "resample particle filter");
    const std::function<void()> update_map_size_func = std::bind(&LocalizationPlugin::updateMapSize, this, std::ref(world));
    const std::function<geo::Transform2()> gen_random_pose_func = std::bind(&LocalizationPlugin::generateRandomPose, this, update_map_size_func);
    particle_filter_.resample(gen_random_pose_func);
    return true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::publishParticles(const ros::Time &stamp)
{
    ROS_DEBUG_NAMED("Localization", "Publishing particles");
    const std::vector<Sample>& samples = particle_filter_.samples();
    geometry_msgs::PoseArray particles_msg;
    particles_msg.poses.resize(samples.size());
    for(unsigned int i = 0; i < samples.size(); ++i)
    {
        const geo::Transform2& p = samples[i].pose;

        geo::Pose3D pose_3d = p.projectTo3d();

        geo::convert(pose_3d, particles_msg.poses[i]);
    }

    particles_msg.header.frame_id = map_frame_id_;
    particles_msg.header.stamp = stamp;

    pub_particles_.publish(particles_msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::updateMapOdom(const geo::Pose3D& odom_to_base_link)
{
    ROS_DEBUG_NAMED("Localization", "Updating map_odom");
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

void LocalizationPlugin::publishMapOdom(const ros::Time &stamp)
{
    ROS_DEBUG_THROTTLE_NAMED(2, "Localization", "Publishing map_odom");
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

TransformStatus LocalizationPlugin::transform(const std::string& target_frame, const std::string& source_frame,
                                              const ros::Time& time, tf2::Stamped<tf2::Transform>& transform)
{
    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_->lookupTransform(target_frame, source_frame, time);
        tf2::convert(ts, transform);
        return OK;
    }
    catch (const tf2::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error
            // (i.e., the scan is too old or too new, respectively)
            geometry_msgs::TransformStamped latest_transform = tf_buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));

            if (scan_buffer_.front()->header.stamp > latest_transform.header.stamp)
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

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    scan_buffer_.push(msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    initial_pose_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::generateRandomPose(std::function<void()> update_map_size)
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

void LocalizationPlugin::updateMapSize(const ed::WorldModel& world)
{
    if (world.revision() <= last_map_size_revision_)
        return;

    geo::Vec2 min(1e6, 1e6), max(-1e6, -1e6);

    for (auto it = world.begin(); it != world.end(); ++ it)
    {
        const ed::EntityConstPtr& e = *it;

        const geo::ShapeConstPtr& shape = e->visual();

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

void LocalizationPlugin::visualize()
{
    ROS_DEBUG_NAMED("Localization", "Visualize");
    int grid_size = 800;
    double grid_resolution = 0.025;

    cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

    std::vector<geo::Vector3> sensor_points;
    laser_model_.renderer().rangesToPoints(laser_model_.sensor_ranges(), sensor_points);

    geo::Transform2 best_pose = (latest_map_odom_ * previous_odom_pose_).projectTo2d();

    geo::Transform2 laser_pose = best_pose * laser_model_.laser_offset();
    for(unsigned int i = 0; i < sensor_points.size(); ++i)
    {
        const geo::Vec2& p = laser_pose * geo::Vec2(sensor_points[i].x, sensor_points[i].y);
        int mx = -(p.y - best_pose.t.y) / grid_resolution + grid_size / 2;
        int my = -(p.x - best_pose.t.x) / grid_resolution + grid_size / 2;

        if (mx >= 0 && my >= 0 && mx < grid_size && my <grid_size)
        {
            rgb_image.at<cv::Vec3b>(my, mx) = cv::Vec3b(0, 255, 0);
        }
    }

    const std::vector<geo::Vec2>& lines_start = laser_model_.lines_start();
    const std::vector<geo::Vec2>& lines_end = laser_model_.lines_end();

    for(unsigned int i = 0; i < lines_start.size(); ++i)
    {
        const geo::Vec2& p1 = lines_start[i];
        int mx1 = -(p1.y - best_pose.t.y) / grid_resolution + grid_size / 2;
        int my1 = -(p1.x - best_pose.t.x) / grid_resolution + grid_size / 2;

        const geo::Vec2& p2 = lines_end[i];
        int mx2 = -(p2.y - best_pose.t.y) / grid_resolution + grid_size / 2;
        int my2 = -(p2.x - best_pose.t.x) / grid_resolution + grid_size / 2;

        cv::line(rgb_image, cv::Point(mx1, my1), cv::Point(mx2, my2), cv::Scalar(255, 255, 255), 1);
    }

    const std::vector<Sample>& samples = particle_filter_.samples();
    for(std::vector<Sample>::const_iterator it = samples.begin(); it != samples.end(); ++it)
    {
        const geo::Transform2& pose = it->pose;

        // Visualize sensor
        int lmx = -(pose.t.y - best_pose.t.y) / grid_resolution + grid_size / 2;
        int lmy = -(pose.t.x - best_pose.t.x) / grid_resolution + grid_size / 2;
        cv::circle(rgb_image, cv::Point(lmx,lmy), 0.1 / grid_resolution, cv::Scalar(0, 0, 255), 1);

        geo::Vec2 d = pose.R * geo::Vec2(0.2, 0);
        int dmx = -d.y / grid_resolution;
        int dmy = -d.x / grid_resolution;
        cv::line(rgb_image, cv::Point(lmx, lmy), cv::Point(lmx + dmx, lmy + dmy), cv::Scalar(0, 0, 255), 1);
    }

    cv::imshow("localization", rgb_image);
    cv::waitKey(1);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
