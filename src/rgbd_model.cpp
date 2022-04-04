#include "rgbd_model.h"

#include "ed_localization/particle_filter.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <tue/profiling/timer.h>

#include <geolib/Mesh.h>
#include <geolib/Shape.h>
#include <geolib/sensors/DepthCamera.h>

#include <rgbd/view.h>

#include <ros/console.h>
#include <opencv2/core/matx.hpp>

#include <vector>

// TEMP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& depth_image, cv::Mat& type_image)
        : geo::RenderResult(depth_image.cols, depth_image.rows), depth_image_(depth_image), type_image_(type_image)
    {
    }

    inline void setType(const unsigned int type) { type_ = type; }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
//        auto& size = depth_image_.size;
//        ROS_WARN_STREAM("Size: " << size[0] << " x " << size[1]);
//        ROS_WARN_STREAM("y: " << y << ", x: " << x);
        float old_depth = depth_image_.at<float>(y, x);
//        ROS_WARN_STREAM("old_depth: " << old_depth << std::endl << "depth: " << depth);
        if (old_depth <= 0 || depth < old_depth)
        {
//            ROS_WARN("RENDER THAT SHIT");
            depth_image_.at<float>(y, x) = depth;
            type_image_.at<unsigned char>(y, x) = type_;
        }
    }

protected:

    cv::Mat& depth_image_;
    cv::Mat& type_image_;
    unsigned int type_;

};

// ----------------------------------------------------------------------------------------------------

bool generateWMImages(const ed::WorldModel& world_model, const geo::DepthCamera& cam, const geo::Pose3D& cam_pose_inv, cv::Mat& depth_image, cv::Mat& type_image, std::vector<std::string>& labels)
{
    SampleRenderResult res(depth_image, type_image);

    geo::RenderOptions opt;

    for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        const std::string& id = e->id().str();

        if (e->shape() && e->has_pose() && (id.size() < 5 || id.substr(id.size() - 5) != "floor") && !e->hasFlag("self")) // Filter ground plane
        {
//            ROS_WARN_STREAM("Rendering: " << id);
            geo::Pose3D pose = cam_pose_inv * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

//            ROS_WARN_STREAM("type: " << e->type());
            const std::string& type = e->type();
            unsigned int index;
            if (type.empty())
            {
                index = UINT8_MAX;
            }
            else
            {
                auto it = std::find(labels.begin(), labels.end(), type);
                index = it-labels.begin();
                if (it == labels.end() && labels.size() < UINT8_MAX)
                    labels.push_back(type);
            }

            // Should be commented/removed for perfomance
//            if (labels.size() >= UINT8_MAX)
//                ROS_ERROR_STREAM("Labels can not be longer than " << UINT8_MAX << ". As it the type image use UINT8");

//            ROS_WARN_STREAM("type index: " << index);
            res.setType(index);

            // Render
            cam.render(opt, res);
        }
    }
    return true;
}

// ----------------------------------------------------------------------------------------------------

RGBDModel::RGBDModel()
{
    labels_.reserve(10);

    // DEFAULT:
    range_max = 10;      // m
}

// ----------------------------------------------------------------------------------------------------

RGBDModel::~RGBDModel()
{
    cv::destroyAllWindows();
}

// ----------------------------------------------------------------------------------------------------

void RGBDModel::configure(tue::Configuration config)
{
    config.value("num_pixels", num_pixels_);

    config.value("range_max", range_max);

    double min_particle_distance;
    config.value("min_particle_distance", min_particle_distance);
    min_particle_distance_sq_ = min_particle_distance * min_particle_distance;

    config.value("min_particle_rotation_distance", min_particle_rotation_distance_);
}

// ----------------------------------------------------------------------------------------------------

bool RGBDModel::updateWeights(const ed::WorldModel& world, std::future<const MaskedImageConstPtr>& masked_image_future, const geo::Pose3D& cam_to_baselink, ParticleFilter& pf)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Find unique samples
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // If N samples are nearly identical, we only want to calculate the probability update
    // once and share it for all N samples. Therefore, we create a sample list 'unique_samples'
    // that only contain samples that are further apart than a given threshold. We will only
    // calculate the probabilities of those samples, and share them with the similar samples.

    // unique samples
    std::vector<geo::Transform2> unique_samples;
    unique_samples.reserve(std::max<uint>(pf.getMaxSamples(), pf.samples().size()));

    // mapping of samples from the particle filter to the unique sample list
    std::vector<unsigned int> sample_to_unique(pf.samples().size());

    for(unsigned int i = 0; i < pf.samples().size(); ++i)
    {
        const Sample& s1 = pf.samples()[i];
        const geo::Transform2& t1 = s1.pose;

        bool found = false;
        for(unsigned int j = 0; j < unique_samples.size(); ++j)
        {
            const geo::Transform2& t2 = unique_samples[j];

            // Calculate difference in rotation
            double rot_diff = std::abs(t1.rotation() - t2.rotation());
            if (rot_diff > M_PI)
                rot_diff = 2 * M_PI - rot_diff;

            // Check if translation and rotational difference are within boundaries
            if ((t1.t - t2.t).length2() < min_particle_distance_sq_ && rot_diff < min_particle_rotation_distance_)
            {
                found = true;
                sample_to_unique[i] = j;
                break;
            }
        }

        if (!found)
        {
            sample_to_unique[i] = unique_samples.size();
            unique_samples.push_back(t1);
        }
    }

    // If there is only one unique sample, it means are particles are (almost) identical, and the laser model
    // update is not neccesary. This typically holds if the robot is standing still.
    if (unique_samples.size() == 1)
    {
        ROS_ERROR("(RGBD) Only one unique sample found");
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Render world model type/depth image
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    MaskedImageConstPtr masked_image;
    if (!cam_.initialized())
    {
         // Getting the msaked image before rendering to get the parameters
         masked_image = masked_image_future.get();
         if (!masked_image)
         {
             ROS_ERROR("(RGBD) Could not get masked image");
             return false;
         }
         cam_ = geo::DepthCamera(masked_image->rgbd_image->getCameraModel());
         size_ = masked_image->rgbd_image->getRGBImage().size();
    }

    std::vector<cv::Mat> depth_images;
    std::vector<cv::Mat> type_images;

    depth_images.resize(unique_samples.size()); // Do not provide default object, as the cv::Mat copy constructor creates a new pointer to the same data
    type_images.resize(unique_samples.size());

    for (uint i = 0; i < unique_samples.size(); ++i)
    {
        const geo::Transform2& sample = unique_samples[i];
        cv::Mat& depth_image = depth_images[i];
        cv::Mat& type_image = type_images[i];
        depth_image = cv::Mat(size_, CV_32FC1, 0.0);
        type_image = cv::Mat(size_, CV_8UC1, UINT8_MAX);

        const geo::Pose3D cam_pose = sample.projectTo3d() * cam_to_baselink;

        bool success = generateWMImages(world, cam_, cam_pose.inverse(), depth_image, type_image, labels_);
    }

    if(!masked_image)
    {
        // If we didn't get the image before to get the camera info
        masked_image = masked_image_future.get();
        if (!masked_image)
        {
            ROS_ERROR("(RGBD) Could not get masked image");
            return false;
        }
    }

    const cv::Mat& sensor_depth_image = masked_image->rgbd_image->getDepthImage();
    const cv::Mat& sensor_type_image = masked_image->mask->image;
    const std::vector<std::string>& sensor_labels = masked_image->labels;

    std::vector<double> weight_updates(unique_samples.size(), 0.);

    int total_pixels = size_.area();
    if (num_pixels_ == 0)
        num_pixels_ = total_pixels;
    uint pixel_step = std::max(total_pixels/num_pixels_, 1); // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult)
    for (uint sample_i = 0; sample_i < unique_samples.size(); ++sample_i)
    {
        const cv::Mat& depth_image = depth_images[sample_i];
        const cv::Mat& type_image = type_images[sample_i];

        double& p = weight_updates[sample_i];
        for (uint i = 0; i<total_pixels; i+=pixel_step)
        {
            uchar sensor_label_index = sensor_type_image.at<uchar>(i);
            uchar label_index = type_image.at<uchar>(i);
            if (sensor_label_index == UINT8_MAX || label_index == UINT8_MAX)
                continue; // Skip pixels that are not labeled
            if (sensor_labels[sensor_label_index] == labels_[label_index])
            {
//                ROS_WARN_STREAM("Sensor label: " << sensor_labels[sensor_label_index] << std::endl << "Sampel label: " << labels_[label_index]);
                ++p;
            }
        }
        p /= num_pixels_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update the particle filter
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::stringstream ss;
    ss << "[";
    for (auto& i : weight_updates)
        ss << i << ", ";
    ss << "]";

    ROS_WARN_STREAM("Unique samples: " << ss.str());
    for(unsigned int j = 0; j < pf.samples().size(); ++j)
    {
        Sample& sample = pf.samples()[j];
        sample.weight *= weight_updates[sample_to_unique[j]];
    }

    pf.normalize(true);

    return true;
}
