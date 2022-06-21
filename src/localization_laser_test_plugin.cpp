#include <exception>
#include "localization_laser_test_plugin.h"

#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_service_options.h>

#include <cv_bridge/cv_bridge.h>

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/Header.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf2_conversions.h>

#include <geometry_msgs/TransformStamped.h>

#include <future>
#include <math.h>

using namespace ed_localization;

// ----------------------------------------------------------------------------------------------------

LocalizationLaserTestPlugin::LocalizationLaserTestPlugin() : laser_offset_initialized_(false)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationLaserTestPlugin::~LocalizationLaserTestPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::configure(tue::Configuration config)
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

    std::string laser_topic;
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
                laser_topic, 1, boost::bind(&LocalizationLaserTestPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationLaserTestPlugin::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    std::string particle_pose_topic;
    if (config.value("particle_pose_topic", particle_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to particle pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
                    particle_pose_topic, 1, boost::bind(&LocalizationLaserTestPlugin::particlePoseCallBack, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_particle_pose_ = nh.subscribe(sub_opts);
    }

    std::string particle_pose_srv;
    if (config.value("particle_pose_srv", particle_pose_srv, tue::config::OPTIONAL))
    {
        // Subscribe to particle pose service
        ros::AdvertiseServiceOptions srv_opts =
                ros::AdvertiseServiceOptions::create<tue_msgs::PoseProbability>(
                    particle_pose_srv, boost::bind(&LocalizationLaserTestPlugin::particlePoseProbCallBack, this, _1, _2), ros::VoidPtr(), &cb_queue_);
        srv_particle_pose_prob_ = nh.advertiseService(srv_opts);
    }

    geo::Transform2d initial_pose = getInitialPose(nh, config);

    geo::Pose3D initial_pose_3d = initial_pose.projectTo3d();
    geometry_msgs::PoseWithCovarianceStamped* msg = new geometry_msgs::PoseWithCovarianceStamped();
    geo::convert(initial_pose_3d, msg->pose.pose);
    msg->header.frame_id = map_frame_id_;
    msg->header.stamp = ros::Time::now();
    initial_pose_msg_ = geometry_msgs::PoseWithCovarianceStampedConstPtr(msg);

    config.value("robot_name", robot_name_);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_ = &world;
    req_ = &req;
    cb_queue_.callAvailable();

    if (initial_pose_msg_)
    {
        // Set initial pose
        geo::Pose3D map_to_base_link;
        geo::convert(initial_pose_msg_->pose.pose, map_to_base_link);
        geometry_msgs::TransformStamped odom_to_base_link_tf;
        TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, ros::Time(0), odom_to_base_link_tf);
        if (ts == OK)
        {
            ROS_WARN_STREAM("Setting initial_pose");
            geo::Pose3D odom_to_base_link;
            geo::convert(odom_to_base_link_tf.transform, odom_to_base_link);
            latest_map_odom_ = map_to_base_link * odom_to_base_link.inverse();
            latest_map_odom_valid_ = true;
            initial_pose_msg_.reset();
        }
    }

    if (particle_pose_msg_)
    {
        if(scan_msg_)
        {
            double prob;
            update(scan_msg_, *particle_pose_msg_, world, req, prob);
            scan_msg_.reset();
        }
        particle_pose_msg_.reset();
    }
    if (latest_map_odom_valid_)
    {
        publishMapOdom(ros::Time::now());
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationLaserTestPlugin::update(const sensor_msgs::LaserScanConstPtr& scan, const geometry_msgs::PoseStamped& pose_msg, const ed::WorldModel& world, ed::UpdateRequest& req, double& prob)
{
    ROS_DEBUG_NAMED("localization", "Updating Laser");

    geometry_msgs::TransformStamped map_msg_frame_tf;
    TransformStatus ts = transform(map_frame_id_, pose_msg.header.frame_id, scan->header.stamp, map_msg_frame_tf);
    if (ts != OK)
    {
        ROS_ERROR_STREAM_NAMED("localization", "Could not transform to global frame: " << map_frame_id_ << ", from: " << base_link_frame_id_);
        return ts;
    }
    geo::Transform3 map_msg_frame;
    geo::convert(map_msg_frame_tf.transform, map_msg_frame);
    geo::Pose3D particle_pose;
    geo::convert(pose_msg.pose, particle_pose);
    particle_pose = map_msg_frame * particle_pose;
//    geo::convert(pose_msg.pose, particle_pose); // Assuming the msg is in map frame

    //  Get transformation from base_link to laser_frame
    if (!laser_offset_initialized_)
    {
        ROS_WARN_NAMED("localization", "Initializing laser offset");
        TransformStatus ts = initLaserOffset(scan->header.frame_id, scan->header.stamp);
        if (ts != OK)
            return ts;
    }

    const geo::Transform2 particle_pose_2d = particle_pose.projectTo2d();

    prob = laser_model_.getParticleProp(world, *scan, particle_pose_2d);

    visualize(particle_pose_2d, prob);

    ROS_INFO_STREAM_NAMED("localization", "Pose: " << particle_pose << std::endl << "resulted in " << prob);

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationLaserTestPlugin::initLaserOffset(const std::string& frame_id, const ros::Time& stamp)
{
    geometry_msgs::TransformStamped p_laser;
    TransformStatus ts = transform(base_link_frame_id_, frame_id, stamp, p_laser);

    if (ts != OK)
        return ts;

    geo::Pose3D offset_3d;
    geo::convert(p_laser.transform, offset_3d);

    geo::Transform2 offset = offset_3d.projectTo2d();

    bool upside_down = offset_3d.getBasis().zz < 0;
    if (upside_down)
    {
        offset.R.yx = -offset.R.yx;
        offset.R.yy = -offset.R.yy;
    }

    double laser_height = offset_3d.getOrigin().getZ();

    laser_model_.setLaserOffset(offset, laser_height, upside_down);

    laser_offset_initialized_ = true;

    return OK;
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationLaserTestPlugin::particlePoseProbCallBack(const tue_msgs::PoseProbabilityRequest& req, tue_msgs::PoseProbabilityResponse& res)
{
    if(!scan_msg_)
    {
        ROS_ERROR("No scan msg");
        return false;
    }

    TransformStatus ts = update(scan_msg_, req.pose, *world_, *req_, res.probability);
    scan_msg_.reset();
    return (ts==OK);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::particlePoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg)
{
    particle_pose_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    scan_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationLaserTestPlugin::visualize(const geo::Transform2& pose, double prob)
{
    ROS_DEBUG_NAMED("localization", "Visualize");
    int grid_size = 800;
    double grid_resolution = 0.025;

    cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

    std::vector<geo::Vector3> sensor_points;
    laser_model_.renderer().rangesToPoints(laser_model_.sensor_ranges(), sensor_points);

    geo::Transform2 best_pose = pose; //(latest_map_odom_ * previous_odom_pose_).projectTo2d();

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

    // Visualize sensor
    int lmx = -(pose.t.y - best_pose.t.y) / grid_resolution + grid_size / 2;
    int lmy = -(pose.t.x - best_pose.t.x) / grid_resolution + grid_size / 2;
    cv::circle(rgb_image, cv::Point(lmx,lmy), 0.1 / grid_resolution, cv::Scalar(0, 0, 255), 1);

    geo::Vec2 d = pose.R * geo::Vec2(0.2, 0);
    int dmx = -d.y / grid_resolution;
    int dmy = -d.x / grid_resolution;
    cv::line(rgb_image, cv::Point(lmx, lmy), cv::Point(lmx + dmx, lmy + dmy), cv::Scalar(0, 0, 255), 1);

    cv::putText(rgb_image, std::to_string(prob), cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

    cv::imshow("localization", rgb_image);
    cv::waitKey(1);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationLaserTestPlugin)
