#ifndef ED_LOCALIZATION_LASER_MODEL_H_
#define ED_LOCALIZATION_LASER_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <tue/config/configuration.h>

#include <sensor_msgs/LaserScan.h>

#include <vector>

namespace ed_localization {

class ParticleFilter;

class LaserModel
{

public:

    LaserModel();

    ~LaserModel();

    void configure(tue::Configuration config);

    void updateWeights(const ed::WorldModel& world, const sensor_msgs::LaserScan& scan, ParticleFilter& pf);

    double getParticleProp(const ed::WorldModel& world, const sensor_msgs::LaserScan& scan, const geo::Transform2& pose);

    const std::vector<geo::Vec2>& lines_start() const { return lines_start_; }
    const std::vector<geo::Vec2>& lines_end() const { return lines_end_; }

    const geo::LaserRangeFinder& renderer() const { return lrf_; }
    const std::vector<double>& sensor_ranges() const { return sensor_ranges_; }

    const geo::Transform2& laser_offset() const { return laser_offset_; }

    void setLaserOffset(const geo::Transform2& offset, double height, bool upside_down)
    {
        laser_offset_ = offset;
        laser_height_ = height;
        laser_upside_down_ = upside_down;
    }

private:

    double z_hit;
    double sigma_hit;
    double z_short;
    double z_max;
    double z_rand;
    double lambda_short;
    double range_max;

    double laser_height_;
    geo::Transform2 laser_offset_;
    bool laser_upside_down_;

    uint num_beams;

    double min_particle_distance_sq_;
    double min_particle_rotation_distance_;

    // CACHING
    std::vector<double> exp_hit_;
    std::vector<double> exp_short_;

    // RENDERING
    geo::LaserRangeFinder lrf_;

    // Visualization
    std::vector<geo::Vec2> lines_start_;
    std::vector<geo::Vec2> lines_end_;
    std::vector<double> sensor_ranges_;

};

}

#endif
