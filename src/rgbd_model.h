#ifndef ED_LOCALIZATION_RGBD_MODEL_H_
#define ED_LOCALIZATION_RGBD_MODEL_H_

#include <ed/types.h>
#include <geolib/sensors/DepthCamera.h>

#include <tue/config/configuration.h>

#include <cv_bridge/cv_bridge.h>
#include <rgbd/types.h>

#include <opencv2/core/types.hpp>

#include <future>

namespace ed_localization {

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

    bool updateWeights(const ed::WorldModel& world, std::future<const MaskedImageConstPtr>& masked_image_future, const geo::Pose3D& base_link_to_cam, ParticleFilter& pf);

    bool generateWMImage(const ed::WorldModel& world, const MaskedImageConstPtr& masked_image_future, const geo::Pose3D& cam_pose_inv, cv::Mat& depth_image, cv::Mat& type_image, std::vector<std::string>& labels);

    double getParticleProp(const cv::Mat& depth_image, const cv::Mat& type_image, const cv::Mat& sensor_depth_image, const cv::Mat& sensor_type_image, const std::vector<std::string>& sensor_labels);

private:

    std::vector<std::string> labels_;

    double range_max_;

    int num_pixels_;

    double min_particle_distance_sq_;
    double min_particle_rotation_distance_;

    // RENDERING
    geo::DepthCamera cam_;
    cv::Size size_;

};

}

#endif
