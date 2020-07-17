#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// SCAN BUFFER
#include <queue>

// TF2
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

// MODELS
#include "particle_filter.h"
#include "odom_model.h"
#include "laser_model.h"

#include <functional>
#include <memory>

namespace tf2 {
    class Transform;
}

namespace tf2_ros {
    class TransformListener;
    class TransformBroadcaster;
}

enum TransformStatus
{
    TOO_RECENT,
    TOO_OLD,
    OK,
    UNKNOWN_ERROR
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

    std::string robot_name_;

    ros::Duration transform_tolerance_;

    // Configuration
    geo::Transform2 getInitialPose(const ros::NodeHandle& nh, tue::Configuration& config);
    geo::Transform2 tryGetInitialPoseFromParamServer(const ros::NodeHandle& nh);
    geo::Transform2 tryGetInitialPoseFromConfig(tue::Configuration& config);

    // PARTICLE FILTER

    ParticleFilter particle_filter_;


    // MODELS

    LaserModel laser_model_;
    OdomModel odom_model_;


    // ROS

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_laser_;

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    bool have_previous_pose_;
    geo::Pose3D previous_pose_;

    ros::Subscriber sub_initial_pose_;

    geometry_msgs::PoseWithCovarianceStampedConstPtr initial_pose_msg_;

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    ros::Publisher pub_particles_;

    bool laser_offset_initialized_;

    // random pose generation

    geo::Vec2 min_map_, max_map_;
    unsigned long last_map_size_revision_;

    geo::Transform2 generateRandomPose(std::function<void()> update_map_size);

    void updateMapSize(const ed::WorldModel& world);

    // Scan buffer
    std::queue<sensor_msgs::LaserScanConstPtr> scan_buffer_;


    // TF2
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    TransformStatus update(const sensor_msgs::LaserScanConstPtr& laser_msg_, const ed::WorldModel& world, ed::UpdateRequest& req);

    TransformStatus transform(const std::string& target_frame, const std::string& source_frame,
                              const ros::Time& time, tf2::Stamped<tf2::Transform>& transform);

};

#endif
