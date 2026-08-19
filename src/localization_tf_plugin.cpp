#include "localization_tf_plugin.h"

#include <ed/plugin.h>
#include <ed/update_request.h>
#include <ed/world_model.h>

#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>

#include <rclcpp/logging.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tue/config/configuration.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

// ----------------------------------------------------------------------------------------------------

LocalizationTFPlugin::LocalizationTFPlugin() = default;

// ----------------------------------------------------------------------------------------------------

LocalizationTFPlugin::~LocalizationTFPlugin() = default;

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::configure(tue::Configuration config)
{
    config.value("robot_name", robot_name_);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationTFPlugin::process(const ed::WorldModel& /*world*/, ed::UpdateRequest& req)
{
    try
    {
        geometry_msgs::msg::TransformStamped const ts =
            tf_buffer_->lookupTransform(robot_name_ + "/base_link", "map", tf2::TimePointZero);

        geo::Pose3D pose;
        geo::convert(ts.transform, pose);

        req.setPose(robot_name_, pose.inverse());
    }
    catch (const tf2::TransformException& exc)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "ED LocalizationTFPlugin: " << exc.what());
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationTFPlugin)
