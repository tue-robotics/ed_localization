#include "rgbd_model.h"

#include "particle_filter.h"

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
    z_hit = 0.95;
    sigma_hit = 0.2;
    z_short = 0.1;
    z_max = 0.05;
    z_rand = 0.05;
    lambda_short = 0.1;
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

    config.value("z_hit", z_hit);
    config.value("sigma_hit", sigma_hit);
    config.value("z_short", z_short);
    config.value("z_max", z_max);
    config.value("z_rand", z_rand);
    config.value("lambda_short", lambda_short);
    config.value("range_max", range_max);
    config.value("min_particle_distance", min_particle_distance_);
    config.value("min_particle_rotation_distance", min_particle_rotation_distance_);

//    // Pre-calculate expensive operations
//    int resolution = 1000; // mm accuracy

//    exp_hit_.resize(range_max * resolution + 1);
//    for(unsigned int i = 0; i < exp_hit_.size(); ++i)
//    {
//        double z = static_cast<double>(i) / resolution;
//        exp_hit_[i] = exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));
//    }

//    exp_short_.resize(range_max * resolution + 1);
//    for(unsigned int i = 0; i < exp_hit_.size(); ++i)
//    {
//        double obs_range = static_cast<double>(i) / resolution;
//        exp_short_[i] = exp(-this->lambda_short * obs_range);
//    }
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
    unique_samples.reserve(pf.getMaxSamples());

    // mapping of samples from the particle filter to the unique sample list
    std::vector<unsigned int> sample_to_unique(pf.samples().size());

    double min_particle_distance_sq = min_particle_distance_ * min_particle_distance_;

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
            if ((t1.t - t2.t).length2() < min_particle_distance_sq && rot_diff < min_particle_rotation_distance_)
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
        return;

    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //    // -     Determine center and maximum range of world model cross section
    //    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    //    // Find the bounding rectangle around all sample poses. This will be used to determine
    //    // the largest render distance (anything max_range beyond the sample boundaries does
    //    // not have to be considered)

    //    geo::Vec2 sample_min(1e9, 1e9);
    //    geo::Vec2 sample_max(-1e9, -1e9);
    //    for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
    //    {
    //        Sample& sample = *it;

    //        geo::Transform2 laser_pose = sample.pose * laser_offset_;

    //        sample_min.x = std::min(sample_min.x, laser_pose.t.x);
    //        sample_min.y = std::min(sample_min.y, laser_pose.t.y);
    //        sample_max.x = std::max(sample_min.x, laser_pose.t.x);
    //        sample_max.y = std::max(sample_min.y, laser_pose.t.y);
    //    }

    //    double temp_range_max = 0;
    //    for(unsigned int i = 0; i < sensor_ranges_.size(); ++i)
    //    {
    //        double r = sensor_ranges_[i];
    //        if (r < range_max)
    //            temp_range_max = std::max(temp_range_max, r);
    //    }

    //    // Add a small buffer to the distance to allow model data that is
    //    // slightly further away to still match
    //    temp_range_max += lambda_short;

    //    // Calculate the sample boundary center and boundary render distance
    //    geo::Vec2 sample_center = (sample_min + sample_max) / 2;
    //    double max_distance = (sample_max - sample_min).length() / 2 + temp_range_max;

    //    // Set the range limit to the lrf renderer. This will make sure all shapes that
    //    // are too far away will not be rendered (object selection, not line selection)
    //    lrf_.setRangeLimits(scan.range_min, max_distance);

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

    depth_images.resize(unique_samples.size(), cv::Mat(size_, CV_32FC1, 0.0));
    type_images.resize(unique_samples.size(), cv::Mat(size_, CV_8UC1, UINT8_MAX));

    for (uint i = 0; i < unique_samples.size(); ++i)
    {
        geo::Transform2& sample = unique_samples[i];
        cv::Mat& depth_image = depth_images[i];
        cv::Mat& type_image = type_images[i];

        geo::Pose3D cam_pose = sample.projectTo3d() * cam_to_baselink;

//        tue::Timer timer;
//        timer.start();
        bool success = generateWMImages(world, cam_, cam_pose.inverse(), depth_image, type_image, labels_);
//        ROS_WARN_STREAM("Rendering took: " << timer.getElapsedTimeInMilliSec() << "ms.");
//        cv::Mat falseColorsMap;
//        cv::applyColorMap(20*type_image, falseColorsMap, cv::COLORMAP_AUTUMN);
//        cv::imshow("out", falseColorsMap);
//        cv::waitKey(1);
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

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    // -     Create world model cross section
//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    geo::Pose3D laser_pose(sample_center.x, sample_center.y, laser_height_);

//    lines_start_.clear();
//    lines_end_.clear();

//    // Give the max_distance also to the line renderer. It will discard all LINES that
//    // are further away. Note that this is an even finer selection than the default
//    // object selection that is done by the lrf renderer.
//    LineRenderResult render_result(lines_start_, lines_end_, max_distance);

//    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
//    {
//        const ed::EntityConstPtr& e = *it;
//        if (e->shape() && e->has_pose())
//        {
//            // Do not render the robot itself (we're trying to localize it!)
//            if (e->hasFlag("self"))
//                continue;

//            if (e->hasFlag("non-localizable"))
//                continue;

//            geo::LaserRangeFinder::RenderOptions options;
//            geo::Transform t_inv = laser_pose.inverse() * e->pose();
//            options.setMesh(e->shape()->getMesh(), t_inv);
//            lrf_.render(options, render_result);
//        }
//    }

//    for(unsigned int i = 0; i < lines_start_.size(); ++i)
//    {
//        lines_start_[i] += sample_center;
//        lines_end_[i] += sample_center;
//    }

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    // -     Calculate sample weight updates
//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    lrf_.setRangeLimits(scan.range_min, temp_range_max);

//    std::vector<double> weight_updates(unique_samples.size());
//    for(unsigned int j = 0; j < unique_samples.size(); ++j)
//    {
//        geo::Transform2 laser_pose = unique_samples[j] * laser_offset_;
//        geo::Transform2 pose_inv = laser_pose.inverse();

//        // Calculate sensor model for this pose
//        std::vector<double> model_ranges(sensor_ranges_.size(), 0);

//        for(unsigned int i = 0; i < lines_start_.size(); ++i)
//        {
//            const geo::Vec2& p1 = lines_start_[i];
//            const geo::Vec2& p2 = lines_end_[i];

//            // Transform the points to the laser pose
//            geo::Vec2 p1_t = pose_inv * p1;
//            geo::Vec2 p2_t = pose_inv * p2;

//            // Render the line as if seen by the sensor
//            lrf_.renderLine(p1_t, p2_t, model_ranges);
//        }

//        double p = 1;

//        for(unsigned int i = 0; i < sensor_ranges_.size(); ++i)
//        {
//            double obs_range = sensor_ranges_[i];
//            double map_range = model_ranges[i];

//            double z = obs_range - map_range;

//            double pz = 0;

//            // Part 1: good, but noisy, hit
//            //            pz += this->z_hit * exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));
//            pz += this->z_hit * exp_hit_[std::min(std::abs(z), range_max) * 1000];

//            // Part 2: short reading from unexpected obstacle (e.g., a person)
//            if(z < 0)
//                //                pz += this->z_short * this->lambda_short * exp(-this->lambda_short*obs_range);
//                pz += this->z_short * this->lambda_short * exp_short_[std::min(obs_range, range_max) * 1000];

//            // Part 3: Failure to detect obstacle, reported as max-range
//            if(obs_range >= this->range_max)
//                pz += this->z_max * 1.0;

//            // Part 4: Random measurements
//            if(obs_range < this->range_max)
//                pz += this->z_rand * 1.0 / this->range_max;

//            // here we have an ad-hoc weighting scheme for combining beam probs
//            // works well, though...
//            p += pz * pz * pz;
//        }

//        weight_updates[j] = p;
//    }


    const cv::Mat& sensor_depth_image = masked_image->rgbd_image->getDepthImage();
    const cv::Mat& sensor_type_image = masked_image->mask->image;
    const std::vector<std::string>& sensor_labels = masked_image->labels;

    std::vector<double> weight_updates(unique_samples.size(), 0.);

    int total_pixels = size_.area();
    if (num_pixels_ == 0)
        num_pixels_ = total_pixels;
    uint pixel_step = std::max(total_pixels/num_pixels_, 1);
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


//    std::srand(unsigned(std::time(nullptr)));
//    std::generate(weight_updates.begin(), weight_updates.end(), std::rand);


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
