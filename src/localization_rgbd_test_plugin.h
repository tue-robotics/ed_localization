#ifndef ED_LOCALIZATION_RGBD_TEST_PLUGIN_H_
#define ED_LOCALIZATION_RGBD_TEST_PLUGIN_H_

#include <ed_localization/localization_plugin_base.h>

#include <geolib/datatypes.h>

// ROS
#include <ros/service_client.h>
#include <ros/subscriber.h>

#include <geometry_msgs/PoseStamped.h>

// RGBD
#include <rgbd/client.h>
#include <rgbd/types.h>

// TF2
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

// MODELS
#include "rgbd_model.h"

#include <functional>
#include <memory>


class LocalizationRGBDTestPlugin : public ed_localization::LocalizationPluginBase
{

public:

    LocalizationRGBDTestPlugin();

    virtual ~LocalizationRGBDTestPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

protected:

    double initial_pose_d_;
    double initial_pose_a_;

    // Particle pose
    geometry_msgs::PoseStampedConstPtr particle_pose_msg_;

    // MODELS
    RGBDModel rgbd_model_;

    // ROS
    ros::ServiceClient masked_image_srv_client_;
    rgbd::Client rgbd_client_;
    ros::Subscriber sub_particle_pose_;

    const MaskedImageConstPtr getMaskedImage(const rgbd::ImageConstPtr& img);

    // Callbacks
    void particlePoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg);

    /**
     * @brief
     * @param img Image Pointer, Can't be nullptr as no checks are performed
     * @param world World Model
     * @param req Update request to be filled
     * @return TransformStatus
     */
    ed_localization::TransformStatus update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req);

};

#endif
