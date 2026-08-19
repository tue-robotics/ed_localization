#ifndef ED_TF_LOCALIZATION_PLUGIN_H_
#define ED_TF_LOCALIZATION_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <memory>

class LocalizationTFPlugin : public ed::Plugin
{

public:
    LocalizationTFPlugin();

    ~LocalizationTFPlugin() override;

    void configure(tue::Configuration config) override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    std::string robot_name_;
};

#endif
