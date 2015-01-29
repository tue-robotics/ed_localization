#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class RelativeLocalizationPlugin : public ed::Plugin
{

public:

    RelativeLocalizationPlugin();

    virtual ~RelativeLocalizationPlugin();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    // LRF

    int num_beams_;

    double range_max_;

    geo::LaserRangeFinder lrf_;

    std::vector<geo::Vec2>& ray_directions_;

    void updateLRFRenderer(const sensor_msgs::LaserScan& scan, std::vector<double>& sensor_ranges);


    // ROS

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_laser_;

    sensor_msgs::LaserScanConstPtr laser_msg_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    bool have_previous_pose_;
    geo::Pose3D previous_pose_;


    // TF

    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;

    tf::TransformListener* tf_listener_;
    tf::TransformBroadcaster* tf_broadcaster_;

};

#endif
