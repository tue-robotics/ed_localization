#ifndef ED_LOCALIZATION_PARTICLE_FILTER_H_
#define ED_LOCALIZATION_PARTICLE_FILTER_H_

#include <geolib/datatypes.h>

struct Sample
{
    geo::Transform2 pose;
    double weight;
};

class ParticleFilter
{

public:

    ParticleFilter();

    ~ParticleFilter();

    std::vector<Sample>& samples() { return samples_; }

private:

    std::vector<Sample> samples_;

};

#endif
