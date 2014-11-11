#ifndef ED_LOCALIZATION_ODOM_MODEL_H_
#define ED_LOCALIZATION_ODOM_MODEL_H_

#include "particle_filter.h"
#include <geolib/datatypes.h>

#include <tue/config/configuration.h>

class OdomModel
{

public:

    OdomModel();

    ~OdomModel();

    void configure(tue::Configuration config);

    void updatePoses(const Transform& movement, double dt, ParticleFilter& pf);

private:

    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    double alpha5;

};

#endif
