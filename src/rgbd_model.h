#ifndef ED_LOCALIZATION_LASER_MODEL_H_
#define ED_LOCALIZATION_LASER_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/LaserRangeFinder.h>

#include <tue/config/configuration.h>

#include <cv_bridge/cv_bridge.h>
#include <rgbd/types.h>

class ParticleFilter;

struct MaskedImage {
    rgbd::ImageConstPtr rgbd_image;
    cv_bridge::CvImageConstPtr mask;
    std::vector<std::string> labels;
};

typedef boost::shared_ptr<MaskedImage> MaskedImagePtr;
typedef boost::shared_ptr<const MaskedImage> MaskedImageConstPtr;

class RGBDModel
{

public:

    RGBDModel();

    ~RGBDModel();

    void configure(tue::Configuration config);

    void updateWeights(const ed::WorldModel& world, const MaskedImageConstPtr& masked_image, const geo::Pose3D& cam_pose_invs, ParticleFilter& pf);

    const std::vector<geo::Vec2>& lines_start() const { return lines_start_; }
    const std::vector<geo::Vec2>& lines_end() const { return lines_end_; }

    const geo::LaserRangeFinder& renderer() const { return lrf_; }
    const std::vector<double>& sensor_ranges() const { return sensor_ranges_; }

private:

    std::vector<std::string> labels_;

    double z_hit;
    double sigma_hit;
    double z_short;
    double z_max;
    double z_rand;
    double lambda_short;
    double range_max;

    int num_beams;

    double min_particle_distance_;
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

#endif
