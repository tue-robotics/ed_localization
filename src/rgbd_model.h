#ifndef ED_LOCALIZATION_LASER_MODEL_H_
#define ED_LOCALIZATION_LASER_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/DepthCamera.h>

#include <tue/config/configuration.h>

#include <cv_bridge/cv_bridge.h>
#include <rgbd/types.h>

#include <opencv2/core/types.hpp>

#include <future>

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

    bool updateWeights(const ed::WorldModel& world, std::future<const MaskedImageConstPtr>& masked_image_future, const geo::Pose3D& cam_to_baselink, ParticleFilter& pf);

    const std::vector<geo::Vec2>& lines_start() const { return lines_start_; }
    const std::vector<geo::Vec2>& lines_end() const { return lines_end_; }

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

    int num_pixels_;

    double min_particle_distance_;
    double min_particle_rotation_distance_;

    // CACHING
    std::vector<double> exp_hit_;
    std::vector<double> exp_short_;

    // RENDERING
    geo::DepthCamera cam_;
    cv::Size size_;

    // Visualization
    std::vector<geo::Vec2> lines_start_;
    std::vector<geo::Vec2> lines_end_;
    std::vector<double> sensor_ranges_;

};

#endif
