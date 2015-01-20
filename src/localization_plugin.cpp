#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() : have_previous_pose_(false), tf_listener_(), tf_broadcaster_(0)
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

    if (config.readGroup("odom_model", tue::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        odom_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("laser_model", tue::REQUIRED))
    {
        config.value("topic", laser_topic);
        laser_model_.configure(config);
        config.endGroup();
    }

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);


    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;

    delete tf_broadcaster_;
    tf_broadcaster_ = new tf::TransformBroadcaster;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    laser_msg_.reset();
    cb_queue_.callAvailable();

    if (!laser_msg_)
        return;

    tue::Timer timer;
    timer.start();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate delta movement based on odom (fetched from TF)
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (!tf_listener_->waitForTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, ros::Duration(1.0)))
    {
        ROS_WARN_STREAM("[ED LOCALIZATION] Cannot get transform from '" << odom_frame_id_ << "' to '" << base_link_frame_id_ << "'.");
        return;
    }

    geo::Pose3D odom_to_base_link;
    Transform movement;

    try
    {
        tf::StampedTransform odom_to_base_link_tf;

        tf_listener_->lookupTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, odom_to_base_link_tf);

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
    }
    catch (tf::TransformException e)
    {
        std::cout << "[ED LOCALIZATION] " << e.what() << std::endl;

        if (!have_previous_pose_)
            return;

        odom_to_base_link = previous_pose_;

        movement.set(geo::Transform2::identity());
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update motion
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    odom_model_.updatePoses(movement, 0, particle_filter_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update sensor
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    laser_model_.updateWeights(world, *laser_msg_, particle_filter_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     (Re)sample
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    particle_filter_.resample(500);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Publish result
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Get the best pose (2D)
    geo::Transform2 p = particle_filter_.bestSample().pose.matrix();

    // Convert best pose to 3D
    geo::Pose3D map_to_base_link;
    map_to_base_link.t = geo::Vector3(p.t.x, p.t.y, 0);
    map_to_base_link.R = geo::Matrix3(p.R.xx, p.R.xy, 0,
                                      p.R.yx, p.R.yy, 0,
                                      0     , 0     , 1);

    geo::Pose3D map_to_odom = map_to_base_link * odom_to_base_link.inverse();

    // Convert to TF transform
    tf::StampedTransform map_to_odom_tf;
    geo::convert(map_to_odom, map_to_odom_tf);

    // Set frame id's and time stamp
    map_to_odom_tf.frame_id_ = map_frame_id_;
    map_to_odom_tf.child_frame_id_ = odom_frame_id_;
    map_to_odom_tf.stamp_ = laser_msg_->header.stamp;

    // Publish TF
    tf_broadcaster_->sendTransform(map_to_odom_tf);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Print profile statistics
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    std::cout << "----------" << std::endl;
//    std::cout << "Total time = " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
//    std::cout << "Time per sample = " << timer.getElapsedTimeInMilliSec() / particle_filter_.samples().size() << " ms" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Visualization
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool visualize = false;
    if (visualize)
    {
        int grid_size = 800;
        double grid_resolution = 0.025;

        cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

        std::vector<geo::Vector3> sensor_points;
        laser_model_.renderer().rangesToPoints(laser_model_.sensor_ranges(), sensor_points);

        geo::Transform2 best_pose = particle_filter_.bestSample().pose.matrix();

        geo::Transform2 laser_pose = best_pose * laser_model_.laser_offset();
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

ED_REGISTER_PLUGIN(LocalizationPlugin)
