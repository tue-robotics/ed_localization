#ifndef ED_LOCALIZATION_LASER_MODEL_H_
#define ED_LOCALIZATION_LASER_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/LaserRangeFinder.h>

class ParticleFilter;

class LaserModel
{

public:

    LaserModel();

    ~LaserModel();

    void updateWeights(const ed::WorldModel& world, const geo::LaserRangeFinder& lrf,
                       const std::vector<double>& sensor_ranges, ParticleFilter& pf);

    const std::vector<geo::Vec2>& lines_start() const { return lines_start_; }
    const std::vector<geo::Vec2>& lines_end() const { return lines_end_; }

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

    // Visualization
    std::vector<geo::Vec2> lines_start_;
    std::vector<geo::Vec2> lines_end_;

};

#endif
