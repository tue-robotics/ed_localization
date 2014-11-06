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
    z_hit = 0.95;
    sigma_hit = 0.2;
    z_short = 0.1;
    z_max = 0.05;
    z_rand = 0.05;
    lambda_short = 0.1;
    range_max = 10;      // m

    laser_height_ = 0.3;
}

// ----------------------------------------------------------------------------------------------------

LaserModel::~LaserModel()
{
}

// ----------------------------------------------------------------------------------------------------

void LaserModel::updateWeights(const ed::WorldModel& world, const geo::LaserRangeFinder& lrf,
                               const std::vector<double>& sensor_ranges, ParticleFilter& pf)
{
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

    std::vector<geo::Vec2> lines_start;
    std::vector<geo::Vec2> lines_end;
    LineRenderResult render_result(lines_start, lines_end);

    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        geo::LaserRangeFinder::RenderOptions options;
        geo::Transform t_inv = laser_pose.inverse() * e->pose();
        options.setMesh(e->shape()->getMesh(), t_inv);
        lrf.render(options, render_result);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate sample weight updates
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    double total_weight = 0;

    for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
    {
        Sample& sample = *it;
        geo::Transform2 pose_inv = sample.pose.matrix().inverse();

        // Calculate sensor model for this pose
        std::vector<double> model_ranges(sensor_ranges.size(), 0);

        for(unsigned int i = 0; i < lines_start.size(); ++i)
        {
            const geo::Vec2& p1 = lines_start[i];
            const geo::Vec2& p2 = lines_end[i];

            // Transform the points to the laser pose
            geo::Vec2 p1_t = pose_inv * p1;
            geo::Vec2 p2_t = pose_inv * p2;

            // Render the line as if seen by the sensor
            lrf.renderLine(p1_t, p2_t, model_ranges);
        }

        double p = 1;

        for(unsigned int i = 0; i < sensor_ranges.size(); ++i)
        {
            double pz = 0.0;

            double obs_range = sensor_ranges[i];
            double map_range = model_ranges[i];

            // Part 1: good, but noisy, hit
            double z = obs_range - map_range;
            pz += this->z_hit * exp(-(z * z) / (2 * this->sigma_hit * this->sigma_hit));

            // Part 2: short reading from unexpected obstacle (e.g., a person)
            if(z < 0)
              pz += this->z_short * this->lambda_short * exp(-this->lambda_short*obs_range);

            // Part 3: Failure to detect obstacle, reported as max-range
            if(obs_range >= this->range_max)
              pz += this->z_max * 1.0;

            // Part 4: Random measurements
            if(obs_range < this->range_max)
              pz += this->z_rand * 1.0 / this->range_max;

            assert(pz <= 1.0);
            assert(pz >= 0.0);

            // here we have an ad-hoc weighting scheme for combining beam probs
            // works well, though...
            p += pz * pz * pz;
        }

        sample.weight *= p;
        total_weight += sample.weight;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Normalize sample weights
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (total_weight > 0)
    {
        for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
            it->weight /= total_weight;
    }
    else
    {
        double uni_weight = 1.0 / pf.samples().size();
        for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
            it->weight = uni_weight;
    }
}


