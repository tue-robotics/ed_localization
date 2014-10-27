#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

struct Sample
{
    geo::Pose3D pose;
};

class LocalizationPlugin : public ed::Plugin
{

public:

    LocalizationPlugin();

    virtual ~LocalizationPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:


    bool pose_initialized_;

    geo::Pose3D best_laser_pose_;

    std::vector<Sample> samples_;

    // RENDERING

    geo::LaserRangeFinder lrf_;

    geo::Pose3D laser_pose_;
    double a_current_;

    // ROS

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_laser_;

    sensor_msgs::LaserScanConstPtr laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    ros::Subscriber sub_odom_;

    nav_msgs::OdometryConstPtr odom_msg_;

    bool have_previous_pose_;
    geo::Pose3D previous_pose_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

};

#endif
