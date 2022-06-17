#ifndef ED_LOCALIZATION_RGBD_TEST_PLUGIN_H_
#define ED_LOCALIZATION_RGBD_TEST_PLUGIN_H_

#include <ed_localization/localization_plugin_base.h>

#include <geolib/datatypes.h>

// ROS
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <tue_msgs/PoseProbability.h>

// RGBD
#include <rgbd/client.h>
#include <rgbd/types.h>

// MODELS
#include "rgbd_model.h"


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
    ed_localization::RGBDModel rgbd_model_;

    // ROS
    ros::ServiceClient masked_image_srv_client_;
    rgbd::Client rgbd_client_;
    ros::Subscriber sub_particle_pose_;
    ros::ServiceServer srv_particle_pose_prob_;

    const ed_localization::MaskedImageConstPtr getMaskedImage(const rgbd::ImageConstPtr& img);

    // Callbacks
    void particlePoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg);
    bool particlePoseProbCallBack(const tue_msgs::PoseProbabilityRequest& req, tue_msgs::PoseProbabilityResponse& res);

    /**
     * @brief
     * @param img Image Pointer, Can't be nullptr as no checks are performed
     * @param world World Model
     * @param req Update request to be filled
     * @return TransformStatus
     */
    ed_localization::TransformStatus update(const rgbd::ImageConstPtr& img, const geometry_msgs::PoseStamped& pose_msg, const ed::WorldModel& world, ed::UpdateRequest& req, double& prob);

    const ed::WorldModel* world_;
    ed::UpdateRequest* req_;
};

#endif
