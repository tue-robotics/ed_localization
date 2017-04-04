#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

#include <geometry_msgs/PoseArray.h>

#include <ed/update_request.h>

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() : have_previous_pose_(false), laser_offset_initialized_(false),
    tf_listener_(), tf_broadcaster_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{
    // Saving last pose in parameter server
    Transform last_pose = particle_filter_.bestSample().pose;

    ros::NodeHandle nh;
    nh.setParam("initialpose/x", last_pose.translation().x);
    nh.setParam("initialpose/y", last_pose.translation().y);
    nh.setParam("initialpose/yaw", last_pose.rotation());
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

    config.value("num_particles", num_particles_);

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::OPTIONAL))
    {
        // Subscribe to initial pose topic

        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationPlugin::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    // Initial pose
    geo::Vec2 p; p.x = 0; p.y = 0;
    double yaw = 0;

    // Getting last pose from parameter server
    std::map<std::string, double> ros_param_position;
    if (nh.getParam("initialpose", ros_param_position))
    {
        p.x = ros_param_position["x"];
        p.y = ros_param_position["y"];
        yaw = ros_param_position["yaw"];
    }
    else if (config.readGroup("initial_pose", tue::OPTIONAL))
    {
        config.value("x", p.x);
        config.value("y", p.y);
        config.value("rz", yaw);
        config.endGroup();
    }

    particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), 0.05,
                                    yaw - 0.1, yaw + 0.1, 0.05);

    config.value("robot_name", robot_name_);

    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;

    delete tf_broadcaster_;
    tf_broadcaster_ = new tf::TransformBroadcaster;

    pub_particles_ = nh.advertise<geometry_msgs::PoseArray>("ed/localization/particles", 10);
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

        double yaw = tf::getYaw(initial_pose_msg_->pose.pose.orientation);

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
        tf::StampedTransform p_laser;
        TransformStatus ts = this->transform(base_link_frame_id_, scan->header.frame_id, scan->header.stamp, p_laser);

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

    tf::StampedTransform odom_to_base_link_tf;
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

    odom_model_.updatePoses(movement, 0, particle_filter_);

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

    particle_filter_.resample(num_particles_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Publish result
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Get the best pose (2D)
    geo::Transform2 mean_pose = particle_filter_.calculateMeanPose();

    // Convert best pose to 3D
    geo::Pose3D map_to_base_link;
    map_to_base_link.t = geo::Vector3(mean_pose.t.x, mean_pose.t.y, 0);
    map_to_base_link.R = geo::Matrix3(mean_pose.R.xx, mean_pose.R.xy, 0,
                                      mean_pose.R.yx, mean_pose.R.yy, 0,
                                      0     , 0     , 1);

    geo::Pose3D map_to_odom = map_to_base_link * odom_to_base_link.inverse();

    // Convert to TF transform
    tf::StampedTransform map_to_odom_tf;
    geo::convert(map_to_odom, map_to_odom_tf);

    // Set frame id's and time stamp
    map_to_odom_tf.frame_id_ = map_frame_id_;
    map_to_odom_tf.child_frame_id_ = odom_frame_id_;
    map_to_odom_tf.stamp_ = scan->header.stamp;

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

    particles_msg.header.frame_id = "/map";
    particles_msg.header.stamp = scan->header.stamp;

    pub_particles_.publish(particles_msg);

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

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::transform(const std::string& target_frame, const std::string& source_frame,
                                              const ros::Time& time, tf::StampedTransform& transform)
{
    try
    {
        tf_listener_->lookupTransform(target_frame, source_frame, time, transform);
        return OK;
    }
    catch(tf::ExtrapolationException& ex)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error
            // (i.e., the scan is too old or too new, respectively)

            tf::StampedTransform latest_transform;
            tf_listener_->lookupTransform(target_frame, source_frame, ros::Time(0), latest_transform);

            if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
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
        catch(tf::TransformException& exc)
        {
            return UNKNOWN_ERROR;
        }
    }
    catch(tf::TransformException& ex)
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

ED_REGISTER_PLUGIN(LocalizationPlugin)
