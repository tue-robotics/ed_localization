#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ed/update_request.h>
#include <ed/world_model.h>

#include <geolib/ros/msg_conversions.h>


using namespace ed_localization;

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() :
    visualize_(false),
    laser_offset_initialized_(false)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationPlugin::configureImpl(tue::Configuration config)
{
    visualize_ = false;
    config.value("visualize", visualize_, tue::config::OPTIONAL);

    std::string laser_topic;
    if (config.readGroup("laser_model", tue::config::REQUIRED))
    {
        config.value("topic", laser_topic);
        laser_model_.configure(config);
        config.endGroup();
    }

    if (config.hasError())
        return false;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::processImpl(const ed::WorldModel& world, ed::UpdateRequest& req)
{
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
        ROS_ERROR_NAMED("localization", "(update) Empty particle filter");
        return UNKNOWN_ERROR;
    }


    // Calculate delta movement based on odom (fetched from TF)
    geo::Pose3D odom_to_base_link;
    geo::Transform2 movement;

    geometry_msgs::TransformStamped odom_to_base_link_tf;
    TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, scan->header.stamp, odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf.transform, odom_to_base_link);

    bool update = false;
    if (have_previous_odom_pose_)
    {
        // Get displacement and project to 2D
        movement = (previous_odom_pose_.inverse() * odom_to_base_link).projectTo2d();

        update = movement.t.x*movement.t.x + movement.t.y*movement.t.y >= update_min_d_sq_ || std::abs(movement.rotation()) >= update_min_a_;
    }

    bool force_publication = false;
    if (!have_previous_odom_pose_)
    {
        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;
        update = true;
        force_publication = true;
    }
    else if (have_previous_odom_pose_ && update)
    {
        // Update motion
        odom_model_.updatePoses(movement, particle_filter_);
    }

    bool resampled = false;
    if (update)
    {
        ROS_DEBUG_NAMED("localization", "Updating laser");
        // Update sensor
        laser_model_.updateWeights(world, *scan, particle_filter_);

        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;

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
        if (cv::getWindowProperty("localization", cv::WND_PROP_AUTOSIZE) >= 0) // Way to check if the window is opened
            cv::destroyWindow("localization");
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::initLaserOffset(const std::string& frame_id, const ros::Time& stamp)
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

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    scan_buffer_.push(msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::visualize()
{
    ROS_DEBUG_NAMED("localization", "Visualize");
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
