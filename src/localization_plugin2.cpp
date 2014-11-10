#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() : have_previous_pose_(false)
{
    particle_filter_.initUniform(geo::Vec2(-1, -5), geo::Vec2(8, 5), 0.2, 0.1);
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::configure(tue::Configuration config)
{
    std::string laser_topic;
    config.value("laser_topic", laser_topic);    

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    std::string odom_topic;
    if (config.value("odom_topic", odom_topic))
    {
        // Subscribe to odometry
        ros::SubscribeOptions sub_odom_options =
                ros::SubscribeOptions::create<nav_msgs::Odometry>(
                    odom_topic, 1, boost::bind(&LocalizationPlugin::odomCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_odom_ = nh.subscribe(sub_odom_options);
    }

    laser_offset_ = geo::Transform2(0.3, 0, 0);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    laser_msg_.reset();
    odom_msg_.reset();
    cb_queue_.callAvailable();

    if (!laser_msg_)
        return;

    tue::Timer timer;
    timer.start();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate delta movement based on odom
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Transform movement;

    if (odom_msg_)
    {
        geo::Pose3D new_pose;
        geo::convert(odom_msg_->pose.pose, new_pose);

        if (have_previous_pose_)
        {

//            prev * delta = new_pose;

            geo::Pose3D delta = previous_pose_.inverse() * new_pose;

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

        previous_pose_ = new_pose;
        have_previous_pose_ = true;
    }
    else
    {
        movement.set(geo::Transform2::identity());
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update world renderer
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int num_beams = laser_msg_->ranges.size();
//    if (poses.size() > 10000)
//        num_beams = 100;  // limit to 100 beams

    int i_step = laser_msg_->ranges.size() / num_beams;
    std::vector<double> sensor_ranges;
    for (unsigned int i = 0; i < laser_msg_->ranges.size(); i += i_step)
    {
        double r = laser_msg_->ranges[i];

        // Check for Inf
        if (r != r || r > laser_msg_->range_max)
            r = 0;
        sensor_ranges.push_back(r);
    }
    num_beams = sensor_ranges.size();

    if (lrf_.getNumBeams() != num_beams)
    {
        lrf_.setNumBeams(num_beams);
        lrf_.setRangeLimits(laser_msg_->range_min, laser_msg_->range_max);
        lrf_.setAngleLimits(laser_msg_->angle_min, laser_msg_->angle_max);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update motion
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    odom_model_.updatePoses(movement, 0, particle_filter_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update sensor
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    laser_model_.updateWeights(world, lrf_, sensor_ranges, particle_filter_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     (Re)sample
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    particle_filter_.resample(500);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Print profile statistics
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::cout << "----------" << std::endl;
    std::cout << "Total time = " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
    std::cout << "Time per sample = " << timer.getElapsedTimeInMilliSec() / particle_filter_.samples().size() << " ms" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Visualization
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool visualize = true;
    if (visualize)
    {
        int grid_size = 800;
        double grid_resolution = 0.025;

        cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

        std::vector<geo::Vector3> sensor_points;
        lrf_.rangesToPoints(sensor_ranges, sensor_points);

        geo::Transform2 best_pose = particle_filter_.bestSample().pose.matrix();

        geo::Transform2 laser_pose = best_pose * laser_offset_;
        for(unsigned int i = 0; i < sensor_points.size(); ++i)
        {
            const geo::Vec2& p = laser_pose * geo::Vec2(sensor_points[i].x, sensor_points[i].y);
            int mx = -p.y / grid_resolution + grid_size / 2;
            int my = -p.x / grid_resolution + grid_size / 2;

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
            int mx1 = -p1.y / grid_resolution + grid_size / 2;
            int my1 = -p1.x / grid_resolution + grid_size / 2;

            const geo::Vec2& p2 = lines_end[i];
            int mx2 = -p2.y / grid_resolution + grid_size / 2;
            int my2 = -p2.x / grid_resolution + grid_size / 2;

            cv::line(rgb_image, cv::Point(mx1, my1), cv::Point(mx2, my2), cv::Scalar(255, 255, 255), 1);
        }

        const std::vector<Sample>& samples = particle_filter_.samples();
        for(std::vector<Sample>::const_iterator it = samples.begin(); it != samples.end(); ++it)
        {
            const geo::Transform2& pose = it->pose.matrix();

            // Visualize sensor
            int lmx = -pose.t.y / grid_resolution + grid_size / 2;
            int lmy = -pose.t.x / grid_resolution + grid_size / 2;
            cv::circle(rgb_image, cv::Point(lmx,lmy), 0.1 / grid_resolution, cv::Scalar(0, 0, 255), 1);

            geo::Vec2 d = pose.R * geo::Vec2(0.2, 0);
            int dmx = -d.y / grid_resolution;
            int dmy = -d.x / grid_resolution;
            cv::line(rgb_image, cv::Point(lmx, lmy), cv::Point(lmx + dmx, lmy + dmy), cv::Scalar(0, 0, 255), 1);
        }

        cv::imshow("localization", rgb_image);
        cv::waitKey(1);
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg_ = msg;
}


// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
