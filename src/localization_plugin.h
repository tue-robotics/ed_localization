#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>

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

    std::vector<Sample> samples_;

    // RENDERING

    geo::LaserRangeFinder lrf_;

    geo::Pose3D laser_pose_;
    double a_current_;

    // ROS

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_laser_;

    sensor_msgs::LaserScanConstPtr last_laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

};

#endif
