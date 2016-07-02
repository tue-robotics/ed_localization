#include "localization_tf_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

#include <geometry_msgs/PoseArray.h>

#include <ed/update_request.h>

// ----------------------------------------------------------------------------------------------------

LocalizationTFPlugin::LocalizationTFPlugin() : tf_listener_()
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationTFPlugin::~LocalizationTFPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::configure(tue::Configuration config)
{
    config.value("robot_name", robot_name_);

    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    try
    {
        tf::StampedTransform t_pose;
        tf_listener_->lookupTransform("/" + robot_name_ + "/base_link", "map", ros::Time(0), t_pose);

        geo::Pose3D pose;
        geo::convert(t_pose, pose);

        req.setPose(robot_name_, pose.inverse());
    }
    catch(tf::TransformException& exc)
    {
        ROS_ERROR_STREAM("ED LocalizationTFPlugin: " << exc.what());
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationTFPlugin)
