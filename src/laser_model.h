#ifndef ED_LOCALIZATION_LASER_MODEL_H_
#define ED_LOCALIZATION_LASER_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <tue/config/configuration.h>

#include <sensor_msgs/LaserScan.h>

class ParticleFilter;

class LaserModel
{

public:

    LaserModel();

    ~LaserModel();

    void configure(tue::Configuration config);

    void updateWeights(const ed::WorldModel& world, const sensor_msgs::LaserScan& scan, ParticleFilter& pf);

    const std::vector<geo::Vec2>& lines_start() const { return lines_start_; }
    const std::vector<geo::Vec2>& lines_end() const { return lines_end_; }

    const geo::LaserRangeFinder& renderer() const { return lrf_; }
    const std::vector<double>& sensor_ranges() const { return sensor_ranges_; }

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

    int num_beams;


    // RENDERING
    geo::LaserRangeFinder lrf_;

    // Visualization
    std::vector<geo::Vec2> lines_start_;
    std::vector<geo::Vec2> lines_end_;
    std::vector<double> sensor_ranges_;

};

#endif
