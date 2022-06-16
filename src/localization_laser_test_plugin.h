#ifndef ED_LOCALIZATION_LASER_TEST_PLUGIN_H_
#define ED_LOCALIZATION_LASER_TEST_PLUGIN_H_

#include <ed_localization/localization_plugin_base.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <sensor_msgs/LaserScan.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <tue_msgs/PoseProbability.h>

// MODELS
#include "laser_model.h"

#include <queue>


class LocalizationLaserTestPlugin : public ed_localization::LocalizationPluginBase
{

public:

    LocalizationLaserTestPlugin();

    virtual ~LocalizationLaserTestPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

protected:

    double initial_pose_d_;
    double initial_pose_a_;

    // Particle pose
    geometry_msgs::PoseStampedConstPtr particle_pose_msg_;

    // MODELS
    ed_localization::LaserModel laser_model_;

    // State
    bool laser_offset_initialized_;

    // Scan buffer
    sensor_msgs::LaserScanConstPtr scan_msg_;

    // ROS
    ros::Subscriber sub_laser_;

    // Init
    ed_localization::TransformStatus initLaserOffset(const std::string& frame_id, const ros::Time& stamp);
    ros::Subscriber sub_particle_pose_;
    ros::ServiceServer srv_particle_pose_prob_;

    // Callbacks
    void particlePoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg);
    bool particlePoseProbCallBack(const tue_msgs::PoseProbabilityRequest& req, tue_msgs::PoseProbabilityResponse& res);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    ed_localization::TransformStatus update(const sensor_msgs::LaserScanConstPtr& scan_msg_, const geometry_msgs::PoseStamped& pose_msg, const ed::WorldModel& world, ed::UpdateRequest& req, double& prob);

    void visualize(const geo::Transform2& pose, double prob);

    const ed::WorldModel* world_;
    ed::UpdateRequest* req_;
};

#endif
