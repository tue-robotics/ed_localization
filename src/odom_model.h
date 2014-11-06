#ifndef ED_LOCALIZATION_ODOM_MODEL_H_
#define ED_LOCALIZATION_ODOM_MODEL_H_

#include "particle_filter.h"
#include <geolib/datatypes.h>

class OdomModel
{

public:

    OdomModel();

    ~OdomModel();

    void updatePoses(const Transform& movement, double dt, ParticleFilter& pf);

private:

    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    double alpha5;

};

#endif
