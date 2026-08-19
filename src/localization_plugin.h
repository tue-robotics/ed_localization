#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// SCAN BUFFER
#include <queue>

// TF2
#include <tf2/transform_datatypes.hpp>

// MODELS
#include "laser_model.h"
#include "odom_model.h"
#include "particle_filter.h"

#include <filesystem>
#include <functional>
#include <memory>

namespace tf2 { class Transform; }

namespace tf2_ros { class TransformBroadcaster; }

enum TransformStatus : uint8_t
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

    ~LocalizationPlugin() override;

    void configure(tue::Configuration config) override;

    void initialize() override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    std::string robot_name_;

    // Config
    bool visualize_;

    int resample_interval_;
    int resample_count_;

    double update_min_d_;
    double update_min_a_;

    // PARTICLE FILTER
    ParticleFilter particle_filter_;

    // MODELS
    LaserModel laser_model_;
    OdomModel odom_model_;

    // Poses
    bool have_previous_odom_pose_;
    geo::Pose3D previous_odom_pose_;

    bool latest_map_odom_valid_;
    geo::Pose3D latest_map_odom_;

    // State
    bool update_;
    bool laser_offset_initialized_;

    // random pose generation
    geo::Vec2 min_map_, max_map_;
    unsigned long last_map_size_revision_;

    // Initial pose
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_;

    // Scan buffer
    std::queue<sensor_msgs::msg::LaserScan::ConstSharedPtr> scan_buffer_;

    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;

    // Persisted pose, recovered on the next run
    std::filesystem::path initial_pose_file_;
    rclcpp::Duration save_pose_interval_;
    rclcpp::Time last_pose_save_;

    // TF2
    rclcpp::Duration transform_tolerance_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;

    // Configuration
    geo::Transform2 getInitialPose(tue::Configuration& config);
    geo::Transform2 tryGetInitialPoseFromParamServer();
    geo::Transform2 tryGetInitialPoseFromFile();
    static geo::Transform2 tryGetInitialPoseFromConfig(tue::Configuration& config);
    void resolveInitialPoseFile(tue::Configuration& config);
    void saveInitialPose();

    // Init
    TransformStatus initLaserOffset(const std::string& frame_id, const rclcpp::Time& stamp);

    void initParticleFilterUniform(const geo::Transform2& pose);

    // Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

    // random pose generation
    geo::Transform2 generateRandomPose(const std::function<void()>& update_map_size);

    void updateMapSize(const ed::WorldModel& world);

    TransformStatus update(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                           const ed::WorldModel& world,
                           ed::UpdateRequest& req);

    bool resample(const ed::WorldModel& world);

    void publishParticles(const rclcpp::Time& stamp);

    void updateMapOdom(const geo::Pose3D& odom_to_base_link);

    void publishMapOdom(const rclcpp::Time& stamp);

    TransformStatus transform(const std::string& target_frame,
                              const std::string& source_frame,
                              const rclcpp::Time& time,
                              tf2::Stamped<tf2::Transform>& transform);

    void visualize();
};

#endif
