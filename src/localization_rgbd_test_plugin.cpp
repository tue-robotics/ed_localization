#include <exception>
#include "localization_rgbd_test_plugin.h"

#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <cv_bridge/cv_bridge.h>

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/Header.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf2_conversions.h>

#include <geometry_msgs/TransformStamped.h>

#include <rgbd/ros/conversions.h>

#include <tue_msgs/GetMaskedImage.h>

#include <visualization_msgs/MarkerArray.h>

#include <future>
#include <math.h>

using namespace ed_localization;

// ----------------------------------------------------------------------------------------------------

LocalizationRGBDTestPlugin::LocalizationRGBDTestPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationRGBDTestPlugin::~LocalizationRGBDTestPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDTestPlugin::configure(tue::Configuration config)
{
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    config.value("resample_interval", resample_interval_);

    config.value("update_min_d", update_min_d_);
    config.value("update_min_a", update_min_a_);

    initial_pose_d_ = 0.3;
    initial_pose_a_ = 0.15;
    config.value("initial_pose_d", initial_pose_d_, tue::config::OPTIONAL);
    config.value("initial_pose_a", initial_pose_a_, tue::config::OPTIONAL);

    std::string rgbd_topic;
    std::string masked_image_srv;

    if (config.readGroup("odom_model", tue::config::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        odom_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("rgbd_model", tue::config::REQUIRED))
    {
        config.value("topic", rgbd_topic);
        config.value("masked_image_srv", masked_image_srv);
        rgbd_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("particle_filter", tue::config::REQUIRED))
    {
        particle_filter_.configure(config);
        config.endGroup();
    }

    double tmp_transform_tolerance = 0.1;
    config.value("transform_tolerance", tmp_transform_tolerance, tue::config::OPTIONAL);
    transform_tolerance_.fromSec(tmp_transform_tolerance);

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationRGBDTestPlugin::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    geo::Transform2d initial_pose = getInitialPose(nh, config);

    config.value("robot_name", robot_name_);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDTestPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDTestPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    initial_pose_msg_.reset();
    cb_queue_.callAvailable();

    if (initial_pose_msg_)
    {
        // Set initial pose
        geo::Pose3D map_to_base_link;
        geo::convert(initial_pose_msg_->pose.pose, map_to_base_link);
        tf2::Stamped<tf2::Transform> odom_to_base_link_tf;
        TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, initial_pose_msg_->header.stamp, odom_to_base_link_tf);
        if (ts == OK)
        {
            geo::Pose3D odom_to_base_link;
            geo::convert(odom_to_base_link_tf, odom_to_base_link);
            latest_map_odom_ = map_to_base_link * odom_to_base_link.inverse();
            latest_map_odom_valid_ = true;
        }
    }

    rgbd::ImagePtr img = rgbd_client_.nextImage();
    if (img)
    {
        update(img, world, req);
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationRGBDTestPlugin::update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Check if particle filter is initialized
    if (particle_filter_.samples().empty())
    {
        ROS_ERROR_NAMED("Localization", "(update) Empty particle filter");
        return UNKNOWN_ERROR;
    }

    auto masked_image_future = std::async(std::launch::async, &LocalizationRGBDTestPlugin::getMaskedImage, this, img);

    // Get transformation from base_link to camera frame
    tf2::Stamped<tf2::Transform> camera_to_base_link_tf;
    TransformStatus ts = transform(base_link_frame_id_, img->getFrameId(), ros::Time(img->getTimestamp()), camera_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::Pose3D camera_to_base_link;
    geo::convert(camera_to_base_link_tf, camera_to_base_link);

    geo::Pose3D rotate180 = geo::Pose3D::identity();
    rotate180.R.setRPY(M_PI, 0, 0);
    camera_to_base_link = camera_to_base_link * rotate180;

    // Calculate delta movement based on odom (fetched from TF)
    geo::Pose3D odom_to_base_link;
    geo::Transform2 movement;

    tf2::Stamped<tf2::Transform> odom_to_base_link_tf;
    ts = transform(odom_frame_id_, base_link_frame_id_, ros::Time(img->getTimestamp()), odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf, odom_to_base_link);

    bool update = false;
    if (have_previous_odom_pose_)
    {
        // Get displacement and project to 2D
        movement = (previous_odom_pose_.inverse() * odom_to_base_link).projectTo2d();

        update = std::abs(movement.t.x) >= update_min_d_ || std::abs(movement.t.y) >= update_min_d_ || std::abs(movement.rotation()) >= update_min_a_;
    }

    bool force_publication = false;
    if (!have_previous_odom_pose_)
    {
        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;
        update = true;
        force_publication = true;
    }
    else if (have_previous_odom_pose_ && update)
    {
        // Update motion
        odom_model_.updatePoses(movement, particle_filter_);
    }

    bool resampled = false;
    if (update)
    {
        ROS_DEBUG_NAMED("Localization", "Updating RGBD");
        // Update sensor
        bool success = rgbd_model_.updateWeights(world, masked_image_future, camera_to_base_link, particle_filter_);
        if (!success)
            return UNKNOWN_ERROR;

        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;

        // (Re)sample
        resampled = resample(world);

        // Publish particles
        publishParticles(ros::Time(img->getTimestamp()));
    }

    // Update map-odom
    if(resampled || force_publication)
    {
        updateMapOdom(odom_to_base_link);
    }

    // Publish result
    if (latest_map_odom_valid_)
    {
        publishMapOdom(ros::Time(img->getTimestamp()));

        // This should be executed allways. map_odom * odom_base_link
        if (!robot_name_.empty())
            req.setPose(robot_name_, latest_map_odom_ * previous_odom_pose_);
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

const MaskedImageConstPtr LocalizationRGBDTestPlugin::getMaskedImage(const rgbd::ImageConstPtr& img)
{
    tue_msgs::GetMaskedImageRequest srv_req;
    tue_msgs::GetMaskedImageResponse srv_resp;
    rgbd::convert(img->getRGBImage(), srv_req.input_image);
    srv_req.input_image.header.frame_id = img->getFrameId();
    srv_req.input_image.header.stamp = ros::Time(img->getTimestamp());
    if (!masked_image_srv_client_.call(srv_req, srv_resp))
    {
        ROS_ERROR("Could not get masked image");
        return nullptr;
    }
    MaskedImagePtr masked_image(new MaskedImage);
    masked_image->rgbd_image = img;
    masked_image->mask = cv_bridge::toCvCopy(srv_resp.output_image.image);
    masked_image->labels = std::move(srv_resp.output_image.labels);

    return masked_image;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationRGBDTestPlugin)
