#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <ros/duration.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// RGBD
#include <rgbd/client.h>
#include <rgbd/types.h>

// TF2
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

// MODELS
#include "particle_filter.h"
#include "odom_model.h"
#include "rgbd_model.h"

#include <functional>
#include <memory>

//namespace cv {
//    typedef std::shared_ptr<Mat> MatPtr;
//    typedef std::shared_ptr<const Mat> MatConstPtr;
//}

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

class LocalizationRGBDPlugin : public ed::Plugin
{

public:

    LocalizationRGBDPlugin();

    virtual ~LocalizationRGBDPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string robot_name_;

    // Config
    bool visualize_;

    int resample_interval_;
    int resample_count_;

    double update_min_d_;
    double update_min_a_;

    double initial_pose_d_;
    double initial_pose_a_;

    // PARTICLE FILTER
    ParticleFilter particle_filter_;

    // MODELS
    RGBDModel rgbd_model_;
    OdomModel odom_model_;

    // Poses
    bool have_previous_odom_pose_;
    geo::Pose3D previous_odom_pose_;

    bool latest_map_odom_valid_;
    geo::Pose3D latest_map_odom_;

    // State
    bool laser_offset_initialized_;

    // random pose generation
    geo::Vec2 min_map_, max_map_;
    unsigned long last_map_size_revision_;

    // Initial pose
    geometry_msgs::PoseWithCovarianceStampedConstPtr initial_pose_msg_;

    // Frames
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;

    // TF2
    ros::Duration transform_tolerance_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS
    ros::CallbackQueue cb_queue_;
    ros::Subscriber sub_initial_pose_;
    ros::Publisher pub_particles_;
    ros::ServiceClient masked_image_srv_client_;
    rgbd::Client rgbd_client_;

    // Configuration
    geo::Transform2 getInitialPose(const ros::NodeHandle& nh, tue::Configuration& config);
    geo::Transform2 tryGetInitialPoseFromParamServer(const ros::NodeHandle& nh);
    geo::Transform2 tryGetInitialPoseFromConfig(tue::Configuration& config);

    // Init
    TransformStatus CameraOffset(const std::string& frame_id, const ros::Time& stamp);

    void initParticleFilterUniform(const geo::Transform2& pose);

    // Callbacks
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    // random pose generation
    geo::Transform2 generateRandomPose(std::function<void()> update_map_size);

    void updateMapSize(const ed::WorldModel& world);

    const MaskedImageConstPtr getMaskedImage(const rgbd::ImageConstPtr& img);

    /**
     * @brief
     * @param img Image Pointer, Can't be nullptr as no checks are performed
     * @param world World Model
     * @param req Update request to be filled
     * @return TransformStatus
     */
    TransformStatus update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req);

    bool resample(const ed::WorldModel& world);

    void publishParticles(const ros::Time& stamp);

    void updateMapOdom(const geo::Pose3D& odom_to_base_link);

    void publishMapOdom(const ros::Time &stamp);

    TransformStatus transform(const std::string& target_frame, const std::string& source_frame,
                              const ros::Time& time, tf2::Stamped<tf2::Transform>& transform);

    void visualize();

};

#endif
