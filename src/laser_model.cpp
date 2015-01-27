#include "laser_model.h"

#include "particle_filter.h"
#include <ed/world_model.h>
#include <ed/entity.h>
#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

class LineRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    LineRenderResult(std::vector<geo::Vec2>& lines_start, std::vector<geo::Vec2>& lines_end)
        : geo::LaserRangeFinder::RenderResult(dummy_ranges_),
          lines_start_(lines_start), lines_end_(lines_end) {}

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2)
    {
        lines_start_.push_back(p1);
        lines_end_.push_back(p2);
    }

private:

    std::vector<double> dummy_ranges_;
    std::vector<geo::Vec2>& lines_start_;
    std::vector<geo::Vec2>& lines_end_;

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

    if (config.readGroup("laser_pose", tue::REQUIRED))
    {
        double x, y, rz;
        config.value("x", x);
        config.value("y", y);
        config.value("rz", rz);
        laser_offset_ = geo::Transform2(x, y, rz);

        config.value("z", laser_height_);

        config.endGroup();
    }

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
        lrf_.setRangeLimits(scan.range_min, scan.range_max);
        lrf_.setAngleLimits(scan.angle_min, scan.angle_max);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create world model cross section
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        if ((*it)->shape())
            entities.push_back(*it);
    }

    geo::Pose3D laser_pose(0, 0, laser_height_);

    lines_start_.clear();
    lines_end_.clear();
    LineRenderResult render_result(lines_start_, lines_end_);

    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        geo::LaserRangeFinder::RenderOptions options;
        geo::Transform t_inv = laser_pose.inverse() * e->pose();
        options.setMesh(e->shape()->getMesh(), t_inv);
        lrf_.render(options, render_result);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate sample weight updates
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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


