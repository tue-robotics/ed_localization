#ifndef ED_LOCALIZATION_PLUGIN_H_
#define ED_LOCALIZATION_PLUGIN_H_

#include <ed_localization/localization_plugin_base.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

// ROS
#include <sensor_msgs/LaserScan.h>

// SCAN BUFFER
#include <queue>

// MODELS
#include "laser_model.h"

#include <functional>
#include <memory>


class LocalizationPlugin : public ed_localization::LocalizationPluginBase
{

public:

    LocalizationPlugin();

    virtual ~LocalizationPlugin();

    virtual bool configureImpl(tue::Configuration config);

    void initialize();

    virtual void processImpl(const ed::WorldModel& world, ed::UpdateRequest& req);

protected:

    // Config
    bool visualize_;

    // MODELS
    LaserModel laser_model_;

    // State
    bool laser_offset_initialized_;

    // Scan buffer
    std::queue<sensor_msgs::LaserScanConstPtr> scan_buffer_;

    // ROS
    ros::Subscriber sub_laser_;

    // Init
    ed_localization::TransformStatus initLaserOffset(const std::string& frame_id, const ros::Time& stamp);

    // Callbacks
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    ed_localization::TransformStatus update(const sensor_msgs::LaserScanConstPtr& laser_msg_, const ed::WorldModel& world, ed::UpdateRequest& req);

    void visualize();

};

#endif
