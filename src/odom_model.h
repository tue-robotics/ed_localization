#ifndef ED_LOCALIZATION_ODOM_MODEL_H_
#define ED_LOCALIZATION_ODOM_MODEL_H_

#include "particle_filter.h"
#include <geolib/datatypes.h>
#include <geolib/math_types.h>

#include <tue/config/configuration.h>

class OdomModel
{

public:

    OdomModel();

    ~OdomModel();

    void configure(tue::Configuration config);

    void updatePoses(const geo::Transform2& movement, ParticleFilter& pf);

private:

    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    double alpha5_;

};

#endif
