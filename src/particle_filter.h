#ifndef ED_LOCALIZATION_PARTICLE_FILTER_H_
#define ED_LOCALIZATION_PARTICLE_FILTER_H_

#include "transform.h"

#include <geolib/datatypes.h>

#include <vector>

// ----------------------------------------------------------------------------------------------------

struct Sample
{
    Sample() {}

    Sample(const geo::Transform2& t)
    {
        pose.set(t);
    }

    double weight;
    Transform pose;
};

// ----------------------------------------------------------------------------------------------------

class ParticleFilter
{

public:

    ParticleFilter();

    ~ParticleFilter();

    void initUniform(const geo::Vec2& min, const geo::Vec2& max, double t_step,
                     double a_min, double a_max, double a_step);

    void resample(unsigned int num_samples = 0);

    std::vector<Sample>& samples() { return samples_[i_current_]; }

    const std::vector<Sample>& samples() const { return samples_[i_current_]; }

    const Sample& bestSample() const;

    geo::Transform2 calculateMeanPose() const;

    void normalize();

private:

    int i_current_;
    std::vector<Sample> samples_[2];

    void setUniformWeights();

};

#endif
