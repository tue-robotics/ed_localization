#include "laser_model.h"

#include "particle_filter.h"
#include <ed/world_model.h>
#include <ed/entity.h>
#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

class LineRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    LineRenderResult(std::vector<geo::Vec2>& lines_start, std::vector<geo::Vec2>& lines_end, double max_distance)
        : geo::LaserRangeFinder::RenderResult(dummy_ranges_),
          lines_start_(lines_start), lines_end_(lines_end), max_distance_sq_(max_distance * max_distance) {}

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2)
    {
        // Calculate distance to the line

        geo::Vec2 diff = p2 - p1;
        double line_length_sq = diff.length2();

        double t = p1.dot(diff) / -line_length_sq;

        double distance_sq;

        if (t < 0)
            distance_sq = p1.length2();
        else if (t > 1)
            distance_sq = p2.length2();
        else
            distance_sq = (p1 + t * diff).length2();

        // If too far, skip
        if (distance_sq > max_distance_sq_)
            return;

        lines_start_.push_back(p1);
        lines_end_.push_back(p2);
    }

private:

    std::vector<double> dummy_ranges_;
    std::vector<geo::Vec2>& lines_start_;
    std::vector<geo::Vec2>& lines_end_;
    double max_distance_sq_;

};

// ----------------------------------------------------------------------------------------------------

LaserModel::LaserModel()
{
    // DEFAULT:
    z_hit = 0.95;
    sigma_hit = 0.2;
    z_short = 0.1;
    z_max = 0.05;
    z_rand = 0.05;
    lambda_short = 0.1;
    range_max = 10;      // m

    laser_height_ = 0.3;
    laser_offset_ = geo::Transform2(0.3, 0, 0);
}

// ----------------------------------------------------------------------------------------------------

LaserModel::~LaserModel()
{
}

// ----------------------------------------------------------------------------------------------------

void LaserModel::configure(tue::Configuration config)
{
    config.value("num_beams", num_beams);

    config.value("z_hit", z_hit);
    config.value("sigma_hit", sigma_hit);
    config.value("z_short", z_short);
    config.value("z_max", z_max);
    config.value("z_rand", z_rand);
    config.value("lambda_short", lambda_short);
    config.value("range_max", range_max);

    // Pre-calculate expensive operations
    int resolution = 1000; // mm accuracy

    exp_hit_.resize(range_max * resolution + 1);
    for(int i = 0; i < exp_hit_.size(); ++i)
    {
        double z = (double)i / resolution;
        exp_hit_[i] = exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));
    }

    exp_short_.resize(range_max * resolution + 1);
    for(int i = 0; i < exp_hit_.size(); ++i)
    {
        double obs_range = (double)i / resolution;
        exp_short_[i] = exp(-this->lambda_short * obs_range);
    }
}

// ----------------------------------------------------------------------------------------------------

void LaserModel::updateWeights(const ed::WorldModel& world, const sensor_msgs::LaserScan& scan, ParticleFilter& pf)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update world renderer
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (num_beams <= 0)
        num_beams = scan.ranges.size();
    else
        num_beams = std::min<int>(scan.ranges.size(), num_beams);

    int i_step = scan.ranges.size() / num_beams;
    sensor_ranges_.clear();
    for (unsigned int i = 0; i < scan.ranges.size(); i += i_step)
    {
        double r = scan.ranges[i];

        // Check for Inf
        if (r != r || r > scan.range_max)
            r = 0;
        sensor_ranges_.push_back(r);
    }
    num_beams = sensor_ranges_.size();

    if (lrf_.getNumBeams() != num_beams)
    {
        lrf_.setNumBeams(num_beams);
        lrf_.setAngleLimits(scan.angle_min, scan.angle_max);
        range_max = std::min<double>(range_max, scan.range_max);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Determine center and maximum range of world model cross section
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Find the bounding rectangle around all sample poses. This will be used to determine
    // the largest render distance (anything max_range beyond the sample boundaries does
    // not have to be considered)

    geo::Vec2 sample_min(1e9, 1e9);
    geo::Vec2 sample_max(-1e9, -1e9);
    for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
    {
        Sample& sample = *it;

        geo::Transform2 laser_pose = sample.pose.matrix() * laser_offset_;

        sample_min.x = std::min(sample_min.x, laser_pose.t.x);
        sample_min.y = std::min(sample_min.y, laser_pose.t.y);
        sample_max.x = std::max(sample_min.x, laser_pose.t.x);
        sample_max.y = std::max(sample_min.y, laser_pose.t.y);
    }

    double temp_range_max = 0;
    for(unsigned int i = 0; i < sensor_ranges_.size(); ++i)
    {
        double r = sensor_ranges_[i];
        if (r < range_max)
            temp_range_max = std::max(temp_range_max, r);
    }

    // Add a small buffer to the distance to allow model data that is
    // slightly further away to still match
    temp_range_max += lambda_short;

    // Calculate the sample boundary center and boundary render distance
    geo::Vec2 sample_center = (sample_min + sample_max) / 2;
    double max_distance = (sample_max - sample_min).length() / 2 + temp_range_max;

    // Set the range limit to the lrf renderer. This will make sure all shapes that
    // are too far away will not be rendered (object selection, not line selection)
    lrf_.setRangeLimits(scan.range_min, max_distance);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create world model cross section
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D laser_pose(sample_center.x, sample_center.y, laser_height_);

    lines_start_.clear();
    lines_end_.clear();

    // Give the max_distance also to the line renderer. It will discard all LINES that
    // are further away. Note that this is an even finer selection than the default
    // object selection that is done by the lrf renderer.
    LineRenderResult render_result(lines_start_, lines_end_, max_distance);

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        if (e->shape())
        {
            geo::LaserRangeFinder::RenderOptions options;
            geo::Transform t_inv = laser_pose.inverse() * e->pose();
            options.setMesh(e->shape()->getMesh(), t_inv);
            lrf_.render(options, render_result);
        }
    }

    for(unsigned int i = 0; i < lines_start_.size(); ++i)
    {
        lines_start_[i] += sample_center;
        lines_end_[i] += sample_center;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate sample weight updates
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    lrf_.setRangeLimits(scan.range_min, temp_range_max);

    for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
    {
        Sample& sample = *it;
\
        geo::Transform2 laser_pose = sample.pose.matrix() * laser_offset_;
        geo::Transform2 pose_inv = laser_pose.inverse();

        // Calculate sensor model for this pose
        std::vector<double> model_ranges(sensor_ranges_.size(), 0);

        for(unsigned int i = 0; i < lines_start_.size(); ++i)
        {
            const geo::Vec2& p1 = lines_start_[i];
            const geo::Vec2& p2 = lines_end_[i];

            // Transform the points to the laser pose
            geo::Vec2 p1_t = pose_inv * p1;
            geo::Vec2 p2_t = pose_inv * p2;

            // Render the line as if seen by the sensor
            lrf_.renderLine(p1_t, p2_t, model_ranges);
        }

        double p = 1;

        for(unsigned int i = 0; i < sensor_ranges_.size(); ++i)
        {
            double obs_range = sensor_ranges_[i];
            double map_range = model_ranges[i];

            double z = obs_range - map_range;

            double pz = 0;

            // Part 1: good, but noisy, hit
//            pz += this->z_hit * exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));
            pz += this->z_hit * exp_hit_[std::min(std::abs(z), range_max) * 1000];

            // Part 2: short reading from unexpected obstacle (e.g., a person)
            if(z < 0)
//                pz += this->z_short * this->lambda_short * exp(-this->lambda_short*obs_range);
                pz += this->z_short * this->lambda_short * exp_short_[std::min(obs_range, range_max) * 1000];

            // Part 3: Failure to detect obstacle, reported as max-range
            if(obs_range >= this->range_max)
                pz += this->z_max * 1.0;

            // Part 4: Random measurements
            if(obs_range < this->range_max)
                pz += this->z_rand * 1.0 / this->range_max;

//            if (pz > 1)
//            {
//                std::cout << "[ED LOCALIZATION] Warning: pz > 1 (pz = " << pz << ")" << std::endl;
//                std::cout << "    obs_range = " << obs_range << ", map_range = " << map_range << std::endl;
//                std::cout << "    hit   = " << this->z_hit * exp_hit_[std::min(std::abs(z), range_max) * 1000] << std::endl;
//                std::cout << "    short = " << this->z_short * this->lambda_short * exp_short_[std::min(obs_range, range_max) * 1000] << std::endl;
//                std::cout << "    max   = " << this->z_max * 1.0 << std::endl;
//                std::cout << "    rand  = " << this->z_rand * 1.0 / this->range_max << std::endl;
//            }

            // here we have an ad-hoc weighting scheme for combining beam probs
            // works well, though...
            p += pz * pz * pz;

        }

        sample.weight *= p;
    }

    pf.normalize();
}


