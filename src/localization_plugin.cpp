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
#include <tf2_ros/transform_listener.h>
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

LocalizationPlugin::LocalizationPlugin() : visualize_(false), have_previous_pose_(false), laser_offset_initialized_(false),
    last_map_size_revision_(0), tf_buffer_(), tf_listener_(nullptr), tf_broadcaster_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
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
        ROS_ERROR("[ED Localization] %s",ex.what());
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::configure(tue::Configuration config)
{
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    visualize_ = false;
    config.value("visualize", visualize_, tue::config::OPTIONAL);

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

    geo::Vec2 p = initial_pose.t;
    double yaw = initial_pose.rotation();
    particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), 0.05,
                                 yaw - 0.1, yaw + 0.1, 0.05);

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
    catch (ConfigurationException)
    {
    }

    try
    {
        return tryGetInitialPoseFromConfig(config);
    }
    catch(ConfigurationException)
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
        std::string msg = "[ED Localization] Could not read initial pose from the parameter server";
        ROS_WARN_STREAM(msg);
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
        std::string msg = "[ED Localization] no transform between odom and base_link";
        ROS_ERROR_STREAM(msg);
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

        ROS_DEBUG_STREAM("[ED Localization] Initial pose from parameter server: [" <<
            result.t.x << ", " << result.t.y << "], yaw:" << result.rotation()
        );
        return result;
    }
    catch (tf2::TransformException ex)
    {
        std::string msg = "[ED Localization] %s" + std::string(ex.what());
        ROS_ERROR_STREAM(msg);
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
        std::string message = "[ED Localization] Initial pose not present in config";
        ROS_WARN_STREAM(message);
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
        geo::Vec2 p(initial_pose_msg_->pose.pose.position.x, initial_pose_msg_->pose.pose.position.y);
        tf2::Quaternion q;
        tf2::convert(initial_pose_msg_->pose.pose.orientation, q);

        double yaw = q.getAngle();

        particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), 0.05,
                                     yaw - 0.1, yaw + 0.1, 0.05);
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
    if (!laser_offset_initialized_)
    {
        tf2::Stamped<tf2::Transform> p_laser;
        TransformStatus ts = transform(base_link_frame_id_, scan->header.frame_id, scan->header.stamp, p_laser);

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
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate delta movement based on odom (fetched from TF)
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D odom_to_base_link;
    Transform movement;

    tf2::Stamped<tf2::Transform> odom_to_base_link_tf;
    TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, scan->header.stamp, odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf, odom_to_base_link);

    if (have_previous_pose_)
    {
        geo::Pose3D delta = previous_pose_.inverse() * odom_to_base_link;

        // Convert to 2D transformation
        geo::Transform2 delta_2d(geo::Mat2(delta.R.xx, delta.R.xy,
                                           delta.R.yx, delta.R.yy),
                                 geo::Vec2(delta.t.x, delta.t.y));

        movement.set(delta_2d);
    }
    else
    {
        movement.set(geo::Transform2::identity());
    }

    previous_pose_ = odom_to_base_link;
    have_previous_pose_ = true;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Check if particle filter is initialized
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (particle_filter_.samples().empty())
        return UNKNOWN_ERROR;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update motion
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    odom_model_.updatePoses(movement, particle_filter_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update sensor
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    tue::Timer timer;
//    timer.start();

    laser_model_.updateWeights(world, *scan, particle_filter_);

//    std::cout << "----------" << std::endl;
//    std::cout << "Number of lines = " << laser_model_.lines_start().size() << std::endl;
//    std::cout << "Total time = " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
//    std::cout << "Time per sample = " << timer.getElapsedTimeInMilliSec() / particle_filter_.samples().size() << " ms" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     (Re)sample
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    const std::function<void()> update_map_size_func = std::bind(&LocalizationPlugin::updateMapSize, this, std::ref(world));
    const std::function<geo::Transform2()> gen_random_pose_func = std::bind(&LocalizationPlugin::generateRandomPose, this, update_map_size_func);
    particle_filter_.resample(gen_random_pose_func);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Publish result
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Get the best pose (2D)
    // Convert best pose to 3D
    geo::Transform2 mean_pose = particle_filter_.calculateMeanPose();

    // Convert best pose to 3D
    geo::Pose3D map_to_base_link;
    map_to_base_link.t = geo::Vector3(mean_pose.t.x, mean_pose.t.y, 0);
    map_to_base_link.R = geo::Matrix3(mean_pose.R.xx, mean_pose.R.xy, 0,
                                      mean_pose.R.yx, mean_pose.R.yy, 0,
                                      0     , 0     , 1);

    geo::Pose3D map_to_odom = map_to_base_link * odom_to_base_link.inverse();

    // Convert to TF transform
    geometry_msgs::TransformStamped map_to_odom_tf;
    geo::convert(map_to_odom, map_to_odom_tf.transform);

    // Set frame id's and time stamp
    map_to_odom_tf.header.frame_id = map_frame_id_;
    map_to_odom_tf.child_frame_id = odom_frame_id_;
    map_to_odom_tf.header.stamp = scan->header.stamp + transform_tolerance_;

    // Publish TF
    tf_broadcaster_->sendTransform(map_to_odom_tf);

    if (!robot_name_.empty())
        req.setPose(robot_name_, map_to_base_link);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Publish particles
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    const std::vector<Sample>& samples = particle_filter_.samples();
    geometry_msgs::PoseArray particles_msg;
    particles_msg.poses.resize(samples.size());
    for(unsigned int i = 0; i < samples.size(); ++i)
    {
        const geo::Transform2& p = samples[i].pose.matrix();

        geo::Pose3D pose_3d;
        pose_3d.t = geo::Vector3(p.t.x, p.t.y, 0);
        pose_3d.R = geo::Matrix3(p.R.xx, p.R.xy, 0,
                                 p.R.yx, p.R.yy, 0,
                                 0     , 0     , 1);

        geo::convert(pose_3d, particles_msg.poses[i]);
    }

    particles_msg.header.frame_id = map_frame_id_;
    particles_msg.header.stamp = scan->header.stamp;

    pub_particles_.publish(particles_msg);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Visualization
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (visualize_)
    {
        int grid_size = 800;
        double grid_resolution = 0.025;

        cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

        std::vector<geo::Vector3> sensor_points;
        laser_model_.renderer().rangesToPoints(laser_model_.sensor_ranges(), sensor_points);

        geo::Transform2 best_pose = mean_pose;

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
            const geo::Transform2& pose = it->pose.matrix();

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
    else
    {
        cv::destroyAllWindows();
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::transform(const std::string& target_frame, const std::string& source_frame,
                                              const ros::Time& time, tf2::Stamped<tf2::Transform>& transform)
{
    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_.lookupTransform(target_frame, source_frame, time);
        tf2::convert(ts, transform);
        return OK;
    }
    catch(tf2::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error
            // (i.e., the scan is too old or too new, respectively)

            geometry_msgs::TransformStamped latest_transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));

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
        catch(tf2::TransformException& exc)
        {
            return UNKNOWN_ERROR;
        }
    }
    catch(tf2::TransformException& ex)
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

ED_REGISTER_PLUGIN(LocalizationPlugin)
