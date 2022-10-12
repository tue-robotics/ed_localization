#include <exception>
#include "localization_rgbd_plugin.h"

#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/subscribe_options.h>

#include <cv_bridge/cv_bridge.h>

#include <ed/world_model.h>
#include <ed/update_request.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/Header.h>

#include <geolib/ros/msg_conversions.h>

#include <rgbd/ros/conversions.h>

#include <tue_msgs/GetMaskedImage.h>

#include <future>
#include <math.h>

using namespace ed_localization;

// ----------------------------------------------------------------------------------------------------

LocalizationRGBDPlugin::LocalizationRGBDPlugin() :
    initial_pose_d_(0),
    initial_pose_a_(0)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationRGBDPlugin::~LocalizationRGBDPlugin()
{
    nh_.shutdown();
    if (publish_map_odom_thread_.joinable())
        publish_map_odom_thread_.join();
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationRGBDPlugin::configureImpl(tue::Configuration config)
{
    initial_pose_d_ = 0.3;
    initial_pose_a_ = 0.15;
    config.value("initial_pose_d", initial_pose_d_, tue::config::OPTIONAL);
    config.value("initial_pose_a", initial_pose_a_, tue::config::OPTIONAL);

    std::string rgbd_topic;
    std::string masked_image_srv;

    if (config.readGroup("rgbd_model", tue::config::REQUIRED))
    {
        config.value("topic", rgbd_topic);
        config.value("masked_image_srv", masked_image_srv);
        rgbd_model_.configure(config);
        config.endGroup();
    }

    if (config.hasError())
        return false;

    rgbd_client_.initialize(ros::names::resolve(rgbd_topic));

    masked_image_srv_client_ = nh_.serviceClient<tue_msgs::GetMaskedImage>(masked_image_srv);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDPlugin::initialize()
{
    publish_map_odom_thread_ = std::thread(&LocalizationRGBDPlugin::publishMapOdomThreadFunc, this, 10);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDPlugin::processImpl(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    rgbd::ImagePtr img = rgbd_client_.nextImage();
    if (img)
    {
        update(img, world, req);
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationRGBDPlugin::update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req)
{
    uint step_counter = 0;
    // Check if particle filter is initialized
    if (particle_filter_.samples().empty())
    {
        ROS_ERROR_NAMED("localization", "(update) Empty particle filter");
        return UNKNOWN_ERROR;
    }

    if (write_csv_)
    {
        std::string csv_file_name;
        csv_file_name.append(std::to_string(loop_counter_));
        csv_file_name.append("-loop_start-");
        csv_file_name.append(std::to_string(step_counter++));
        csv_file_name.append(".csv");
        particle_filter_.writeCSV(csv_file_name);
    }

    auto masked_image_future = std::async(std::launch::async, &LocalizationRGBDPlugin::getMaskedImage, this, img);

    // Get transformation from base_link to camera frame
    geometry_msgs::TransformStamped base_link_to_camera_tf;
    TransformStatus ts = transform(base_link_frame_id_, img->getFrameId(), ros::Time(img->getTimestamp()), base_link_to_camera_tf);
    if (ts != OK)
        return ts;

    geo::Pose3D base_link_to_camera;
    geo::convert(base_link_to_camera_tf.transform, base_link_to_camera);

    geo::Pose3D rotate180 = geo::Pose3D::identity();
    rotate180.R.setRPY(M_PI, 0, 0);
    base_link_to_camera = base_link_to_camera * rotate180;

    // Calculate delta movement based on odom (fetched from TF)
    geo::Pose3D odom_to_base_link;
    geo::Transform2 movement;

    geometry_msgs::TransformStamped odom_to_base_link_tf;
    ts = transform(odom_frame_id_, base_link_frame_id_, ros::Time(img->getTimestamp()), odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf.transform, odom_to_base_link);

    bool update = false;
    if (have_previous_odom_pose_)
    {
        // Get displacement and project to 2D
        movement = (previous_odom_pose_.inverse() * odom_to_base_link).projectTo2d();

        update = movement.t.x*movement.t.x + movement.t.y*movement.t.y >= update_min_d_sq_ || std::abs(movement.rotation()) >= update_min_a_;
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
        if (write_csv_)
        {
            std::string csv_file_name;
            csv_file_name.append(std::to_string(loop_counter_));
            csv_file_name.append("-odom_update-");
            csv_file_name.append(std::to_string(step_counter++));
            csv_file_name.append(".csv");
            particle_filter_.writeCSV(csv_file_name);
        }
    }

    bool resampled = false;
    if (update)
    {
        ROS_DEBUG_NAMED("localization", "Updating RGBD");
        // Update sensor
        bool success = rgbd_model_.updateWeights(world, masked_image_future, base_link_to_camera, particle_filter_);
        if (!success)
            return UNKNOWN_ERROR;

        if (write_csv_)
        {
            std::string csv_file_name;
            csv_file_name.append(std::to_string(loop_counter_));
            csv_file_name.append("-rgbd_update-");
            csv_file_name.append(std::to_string(step_counter++));
            csv_file_name.append(".csv");
            particle_filter_.writeCSV(csv_file_name);
        }

        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;

        // (Re)sample
        resampled = resample(world);
        if (resampled && write_csv_)
        {
            std::string csv_file_name;
            csv_file_name.append(std::to_string(loop_counter_));
            csv_file_name.append("-resampled-");
            csv_file_name.append(std::to_string(step_counter++));
            csv_file_name.append(".csv");
            particle_filter_.writeCSV(csv_file_name);
        }

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
        // This should be executed allways. map_odom * odom_base_link
        if (!robot_name_.empty())
            req.setPose(robot_name_, latest_map_odom_ * previous_odom_pose_);
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationRGBDPlugin::publishMapOdomThreadFunc(const float frequency)
{
    ros::Rate r(frequency);
    while(nh_.ok())
    {
        if (latest_map_odom_valid_)
        {
            publishMapOdom(ros::Time::now());
        }
        r.sleep();
    }
}

// ----------------------------------------------------------------------------------------------------

const MaskedImageConstPtr LocalizationRGBDPlugin::getMaskedImage(const rgbd::ImageConstPtr& img)
{
    ROS_DEBUG_STREAM_NAMED("localization", "Prepairing for a masked image service call");
    tue_msgs::GetMaskedImageRequest srv_req;
    tue_msgs::GetMaskedImageResponse srv_resp;
    rgbd::convert(img->getRGBImage(), srv_req.input_image);
    srv_req.input_image.header.frame_id = img->getFrameId();
    srv_req.input_image.header.stamp = ros::Time(img->getTimestamp());
    ROS_DEBUG_STREAM_NAMED("localization", "Calling masked image service");
    if (!masked_image_srv_client_.call(srv_req, srv_resp))
    {
        ROS_ERROR("Could not get masked image");
        return nullptr;
    }
    ROS_DEBUG_STREAM_NAMED("localization", "Receive a masked image");
    MaskedImagePtr masked_image(new MaskedImage);
    masked_image->rgbd_image = img;
    masked_image->mask = cv_bridge::toCvCopy(srv_resp.output_image.image);
    masked_image->labels = std::move(srv_resp.output_image.labels);
    ROS_DEBUG_STREAM_NAMED("localization", "Processed the masked image");

    return masked_image;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationRGBDPlugin)
