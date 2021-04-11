#ifndef ED_TF_LOCALIZATION_PLUGIN_H_
#define ED_TF_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <tf2_ros/buffer.h>

#include <memory>

namespace tf2_ros {
    class TransformListener;
}

class LocalizationTFPlugin : public ed::Plugin
{

public:

    LocalizationTFPlugin();

    virtual ~LocalizationTFPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string robot_name_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

};

#endif
