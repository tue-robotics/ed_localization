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

    double update_min_d;
    config.value("update_min_d", update_min_d);
    update_min_d_sq_ = update_min_d * update_min_d;
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

    rgbd_client_.initialize(ros::names::resolve(rgbd_topic));

    ros::NodeHandle nh;

    masked_image_srv_client_ = nh.serviceClient<tue_msgs::GetMaskedImage>(masked_image_srv);

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(
                    initial_pose_topic, 1, boost::bind(&LocalizationRGBDTestPlugin::initialPoseCallback, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_initial_pose_ = nh.subscribe(sub_opts);
    }

    std::string particle_pose_topic;
    if (config.value("particle_pose_topic", particle_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to particle pose topic
        ros::SubscribeOptions sub_opts =
                ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
                    particle_pose_topic, 1, boost::bind(&LocalizationRGBDTestPlugin::particlePoseCallBack, this, _1), ros::VoidPtr(), &cb_queue_);
        sub_particle_pose_ = nh.subscribe(sub_opts);
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
    cb_queue_.callAvailable();

    if (initial_pose_msg_)
    {
        // Set initial pose
        geo::Pose3D map_to_base_link;
        geo::convert(initial_pose_msg_->pose.pose, map_to_base_link);
        geometry_msgs::TransformStamped odom_to_base_link_tf;
        TransformStatus ts = transform(odom_frame_id_, base_link_frame_id_, ros::Time(0), odom_to_base_link_tf);
        if (ts == OK)
        {
            geo::Pose3D odom_to_base_link;
            geo::convert(odom_to_base_link_tf.transform, odom_to_base_link);
            latest_map_odom_ = map_to_base_link * odom_to_base_link.inverse();
            latest_map_odom_valid_ = true;
            initial_pose_msg_.reset();
        }
    }

    if (particle_pose_msg_)
    {
        rgbd::ImagePtr img = rgbd_client_.nextImage();
        if (img)
        {
            update(img, world, req);
        }
        particle_pose_msg_.reset();
    }
    if (latest_map_odom_valid_)
    {
        publishMapOdom(ros::Time::now());
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationRGBDTestPlugin::update(const rgbd::ImageConstPtr& img, const ed::WorldModel& world, ed::UpdateRequest& req)
{
    ROS_DEBUG_NAMED("localization", "Updating RGBD");
    auto masked_image_future = std::async(std::launch::async, &LocalizationRGBDTestPlugin::getMaskedImage, this, img);

    geo::Pose3D particle_pose;
//    geometry_msgs::TransformStamped particle_pose_tf;
//    TransformStatus ts = transform(base_link_frame_id_, map_frame_id_, ros::Time(img->getTimestamp()), particle_pose_tf);
//    if (ts != OK)
//    {
//        ROS_ERROR_STREAM_NAMED("localization", "Could not transform to global frame: " << map_frame_id_ << ", from: " << base_link_frame_id_);
//        return ts;
//    }
//    geo::convert(particle_pose_tf.transform, particle_pose);
    geo::convert(particle_pose_msg_->pose, particle_pose); // Assuming the msg is in map frame

    // Get transformation from base_link to camera frame
    geometry_msgs::TransformStamped base_link_to_camera_tf;
    TransformStatus ts = transform(base_link_frame_id_, img->getFrameId(), ros::Time(img->getTimestamp()), base_link_to_camera_tf);
    if (ts != OK)
    {
        ROS_ERROR("NOT OK");
        return ts;
    }

    geo::Pose3D base_link_to_camera;
    geo::convert(base_link_to_camera_tf.transform, base_link_to_camera);

    geo::Pose3D rotate180 = geo::Pose3D::identity();
    rotate180.R.setRPY(M_PI, 0, 0);
    base_link_to_camera = base_link_to_camera * rotate180;

    // Variables
    cv::Mat depth_image;
    cv::Mat type_image;
    std::vector<std::string> labels;

    const MaskedImageConstPtr masked_image = masked_image_future.get();
    if (!masked_image)
    {
        ROS_ERROR_NAMED("localization", "Could not get masked image");
        return UNKNOWN_ERROR;
    }

    bool success = rgbd_model_.generateWMImage(world, masked_image, (particle_pose * base_link_to_camera).inverse(), depth_image, type_image, labels);
    if (!success)
    {
        ROS_ERROR_NAMED("localization", "Could not render WM images");
        return UNKNOWN_ERROR;
    }

    double prob = rgbd_model_.getParticleProp(depth_image, type_image, masked_image->rgbd_image->getDepthImage(), masked_image->mask->image, masked_image->labels);

    cv::Size size = masked_image->rgbd_image->getRGBImage().size();

    cv::Mat canvas = cv::Mat(size.height, size.width * 2, CV_8UC1, UINT8_MAX);

    cv::Mat sensor_roi = canvas(cv::Rect(cv::Point(0, 0), size));
    cv::Mat wm_roi = canvas(cv::Rect(cv::Point(size.width, 0), size));

    cv::resize(30*masked_image->mask->image, sensor_roi, size);
    cv::resize(30*type_image, wm_roi, size);

    cv::putText(canvas, std::to_string(prob), cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255), 1);

    cv::namedWindow("type_image");
    cv::imshow("type_image", canvas);
    cv::waitKey(1);

    ROS_ERROR_STREAM_NAMED("localization", "Pose: " << particle_pose << std::endl << "resulted in " << prob);

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

void LocalizationRGBDTestPlugin::particlePoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg)
{
    particle_pose_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationRGBDTestPlugin)
