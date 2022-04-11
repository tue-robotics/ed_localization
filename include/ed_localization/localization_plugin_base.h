#ifndef ED_LOCALIZATION_PLUGIN_BASE_H_
#define ED_LOCALIZATION_PLUGIN_BASE_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>

// ROS
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

// TF2
#include <tf2_ros/buffer.h>

// MODELS
#include "particle_filter.h"
#include "odom_model.h"

#include <exception>
#include <functional>
#include <memory>

namespace tf2_ros {
    class TransformListener;
    class TransformBroadcaster;
}

namespace ed_localization
{

enum TransformStatus
{
    TOO_RECENT,
    TOO_OLD,
    OK,
    UNKNOWN_ERROR
};

class ConfigurationException: public std::exception
{
public:
    ConfigurationException(const std::string& msg){message_ = msg;}

    virtual const char* what() const throw()
    {
        return message_.c_str();
    }
private:
    std::string message_;
};

class LocalizationPluginBase : public ed::Plugin
{

public:

    LocalizationPluginBase();

    /**
     * @brief Stores the latest map-odom transform on the parameterserver under "initial_pose".
     * Which is retrieved by LocalizationPluginBase::tryGetInitialPoseFromParamServer,
     * which is called by LocalizationPluginBase::configure.
     */
    virtual ~LocalizationPluginBase();

    /**
     * @brief Will check all configuration values. Without any error, it will call LocalizationPluginBase::configureImpl. When that returns succesfully,
     * it will create ROS connections.
     * @param config configuration object
     */
    void configure(tue::Configuration config);

    /**
     * @brief Derived classes should first check all configuration values. In case of an error, return false.
     * Otherwise create ROS connections and return true.
     * @param config configuration object
     * @return success
     */
    virtual bool configureImpl(tue::Configuration config);

    void initialize();

    /**
     * @brief Will clear the most recent intial pose message before calling the callback queue for new messages.
     * In case of an initial pose message, the partical filter is re-initialized with the new intial pose.
     * After which ed_localization::LocalizationPluginBase::processImpl is called.
     * @param world World reference
     * @param req Update request
     */
    virtual void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    /**
     * @brief This is called by ed_localization::LocalizationPluginBase::process after it has check for a new initial pose message
     * and has called the callback queue to process any new messages and/or services.
     * @param world World reference
     * @param req update request
     */
    virtual void processImpl(const ed::WorldModel& world, ed::UpdateRequest& req);

protected:

    std::string robot_name_;

    // Config
    int resample_interval_;
    int resample_count_;

    double update_min_d_sq_;
    double update_min_a_;

    // PARTICLE FILTER
    ParticleFilter particle_filter_;

    // Models
    OdomModel odom_model_;

    // Poses
    bool have_previous_odom_pose_;
    geo::Pose3D previous_odom_pose_;

    bool latest_map_odom_valid_;
    geo::Pose3D latest_map_odom_;

    // random pose generation
    geo::Vec2 min_map_, max_map_;
    unsigned long last_map_size_revision_;

    // Initial pose
    geometry_msgs::PoseWithCovarianceStampedConstPtr initial_pose_msg_;

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

    // Configuration
    geo::Transform2 getInitialPose(const ros::NodeHandle& nh, tue::Configuration& config);
    geo::Transform2 tryGetInitialPoseFromParamServer(const ros::NodeHandle& nh);
    geo::Transform2 tryGetInitialPoseFromConfig(tue::Configuration& config);

    // Init
    TransformStatus initLaserOffset(const std::string& frame_id, const ros::Time& stamp);

    void initParticleFilterUniform(const geo::Transform2& pose);

    // Callbacks
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    // random pose generation
    geo::Transform2 generateRandomPose(std::function<void()> update_map_size);

    void updateMapSize(const ed::WorldModel& world);

    bool resample(const ed::WorldModel& world);

    void publishParticles(const ros::Time& stamp);
    uint old_msg_size_;

    void updateMapOdom(const geo::Pose3D& odom_to_base_link);

    void publishMapOdom(const ros::Time &stamp);

    TransformStatus transform(const std::string& target_frame, const std::string& source_frame,
                              const ros::Time& time, geometry_msgs::TransformStamped& transform);

};

} // END NS ed_localization

#endif
