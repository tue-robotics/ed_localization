#ifndef ED_LOCALIZATION_RGBD_PLUGIN_H_
#define ED_LOCALIZATION_RGBD_PLUGIN_H_

#include <ed_localization/localization_plugin_base.h>

#include <geolib/datatypes.h>

// ROS
#include <ros/service_client.h>

// RGBD
#include <rgbd/client.h>
#include <rgbd/types.h>

// TF2
#include <tf2/transform_datatypes.h>

// MODELS
#include "ed_localization/particle_filter.h"
#include "ed_localization/odom_model.h"
#include "rgbd_model.h"

#include <functional>
#include <memory>


class LocalizationRGBDPlugin : public ed_localization::LocalizationPluginBase
{

public:

    LocalizationRGBDPlugin();

    virtual ~LocalizationRGBDPlugin();

    bool configureImpl(tue::Configuration config);

    void initialize();

    void processImpl(const ed::WorldModel& world, ed::UpdateRequest& req);

protected:

    double initial_pose_d_;
    double initial_pose_a_;

    // MODELS
    RGBDModel rgbd_model_;

    // ROS
    ros::ServiceClient masked_image_srv_client_;
    rgbd::Client rgbd_client_;

    const MaskedImageConstPtr getMaskedImage(const rgbd::ImageConstPtr& img);

    /**
     * @brief
     * @param img Image Pointer, Can't be nullptr as no checks are performed
     * @param world World Model
     * @param req Update request to be filled
     * @return TransformStatus
     */
    ed_localization::TransformStatus update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req);

    uint old_msg_size_;

    void publishParticles(const ros::Time& stamp);

};

#endif
