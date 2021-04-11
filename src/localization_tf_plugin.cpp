#include "localization_tf_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>

#include <geolib/ros/msg_conversions.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>

// ----------------------------------------------------------------------------------------------------

LocalizationTFPlugin::LocalizationTFPlugin() : tf_buffer_(), tf_listener_(nullptr)
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
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::initialize()
{
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::process(const ed::WorldModel& /*world*/, ed::UpdateRequest& req)
{
    try
    {
        geometry_msgs::TransformStamped ts = tf_buffer_.lookupTransform(robot_name_ + "/base_link", "map", ros::Time(0));

        geo::Pose3D pose;
        geo::convert(ts.transform, pose);

        req.setPose(robot_name_, pose.inverse());
    }
    catch(tf2::TransformException& exc)
    {
        ROS_ERROR_STREAM("ED LocalizationTFPlugin: " << exc.what());
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationTFPlugin)
