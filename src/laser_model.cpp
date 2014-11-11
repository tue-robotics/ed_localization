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
          lines_start_(lines_start), lines_end_(lines_end), p_min(1e9), p_max(-1e9) {}

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2)
    {
        lines_start_.push_back(p1);
        lines_end_.push_back(p2);

        p_min.x = std::min(p_min.x, std::min(p1.x, p2.x));
        p_max.x = std::max(p_max.x, std::max(p1.x, p2.x));

        p_min.y = std::min(p_min.y, std::min(p1.y, p2.y));
        p_max.y = std::max(p_max.y, std::max(p1.y, p2.y));
    }

private:

    std::vector<double> dummy_ranges_;
    std::vector<geo::Vec2>& lines_start_;
    std::vector<geo::Vec2>& lines_end_;

public:

    geo::Vec2 p_min, p_max;

};

// ----------------------------------------------------------------------------------------------------

LaserModel::LaserModel()
{
    // DEFAULT:
//    z_hit = 0.95;
//    sigma_hit = 0.2;
//    z_short = 0.1;
//    z_max = 0.05;
//    z_rand = 0.05;
//    lambda_short = 0.1;
//    range_max = 10;      // m

    z_hit = 1;
    sigma_hit = 0.2;
    z_short = 0;
    z_max = 0;
    z_rand = 0;
    lambda_short = 0;
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
        if (it->second->shape())
            entities.push_back(it->second);
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

        double p = 0;

        for(unsigned int i = 0; i < sensor_ranges_.size(); ++i)
        {
            double obs_range = sensor_ranges_[i];
            double map_range = model_ranges[i];

            double z = obs_range - map_range;

            double pz = 0;

            if (obs_range > 0 && map_range > 0)
            {
                double z_abs = std::abs(z);

                if (z_abs < 0.1)
                {
                    pz = 1.0 / sensor_ranges_.size();
                }
                else if (z_abs < 0.2)
                {
                    pz = 0.5 / sensor_ranges_.size();
                }
                else if (z_abs < 0.3)
                {
                    pz = 0.25 / sensor_ranges_.size();
                }
            }

            p += pz;

//            // Part 1: good, but noisy, hit
//            pz += this->z_hit * exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));

//            // Part 2: short reading from unexpected obstacle (e.g., a person)
//            if(z < 0)
//                pz += this->z_short * this->lambda_short * exp(-this->lambda_short*obs_range);

//            // Part 3: Failure to detect obstacle, reported as max-range
//            if(obs_range >= this->range_max)
//                pz += this->z_max * 1.0;

//            // Part 4: Random measurements
//            if(obs_range < this->range_max)
//                pz += this->z_rand * 1.0 / this->range_max;

//            assert(pz <= 1.0);
//            assert(pz >= 0.0);

            // here we have an ad-hoc weighting scheme for combining beam probs
            // works well, though...
//            p += pz * pz * pz;
        }

        sample.weight *= p;
    }

    pf.normalize();
}


