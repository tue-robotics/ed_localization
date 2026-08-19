#include "odom_model.h"

#include "particle_filter.h"

#include <geolib/math_types.h>

#include <tue/config/configuration.h>

#include <cmath>
// drand48() is POSIX, declared by <stdlib.h>; <cstdlib> only guarantees the ISO C subset.
#include <stdlib.h> // NOLINT(modernize-deprecated-headers)

// ----------------------------------------------------------------------------------------------------

namespace
{

//! drand48() may return exactly 0.0, which the Box-Muller transform below cannot use.
double nonZeroUniform()
{
    double r = drand48();
    while (r == 0.0)
        r = drand48();

    return r;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double generateRandomGaussian(double sigma)
{
    double x1 = 0.0;
    double x2 = 0.0;
    // Rejection-sample a point inside the unit circle, excluding the origin. w == 0.0 initially, so
    // the loop always runs at least once.
    double w = 0.0;
    while (w > 1.0 || w == 0.0)
    {
        x1 = (2.0 * nonZeroUniform()) - 1.0;
        x2 = (2.0 * nonZeroUniform()) - 1.0;
        w = (x1 * x1) + (x2 * x2);
    }

    return sigma * x2 * sqrt(-2.0 * log(w) / w);
}

} // namespace

// ----------------------------------------------------------------------------------------------------

OdomModel::OdomModel() : alpha1_(0.2), alpha2_(0.2), alpha3_(0.2), alpha4_(0.2), alpha5_(0.2) {}

// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------

void OdomModel::configure(tue::Configuration config)
{
    config.value("alpha1", alpha1_);
    config.value("alpha2", alpha2_);
    config.value("alpha3", alpha3_);
    config.value("alpha4", alpha4_);
    config.value("alpha5", alpha5_);
}

// ----------------------------------------------------------------------------------------------------

void OdomModel::updatePoses(const geo::Transform2& movement, ParticleFilter& pf) const
{
    double const delta_trans_sq = movement.t.length2();

    double const delta_rot = movement.rotation();
    double const delta_rot_sq = delta_rot * delta_rot;

    // Compute noise standard deviations
    double const trans_hat_stddev = sqrt((alpha3_ * delta_trans_sq) + (alpha4_ * delta_rot_sq));
    double const rot_hat_stddev = sqrt((alpha1_ * delta_rot_sq) + (alpha2_ * delta_trans_sq));
    double const strafe_hat_stddev = sqrt((alpha4_ * delta_rot_sq) + (alpha5_ * delta_trans_sq));

    for (auto& sample : pf.samples())
    {
        // Sample pose differences
        double const delta_trans_hat = generateRandomGaussian(trans_hat_stddev);
        double const delta_rot_hat = generateRandomGaussian(rot_hat_stddev);
        double const delta_strafe_hat = generateRandomGaussian(strafe_hat_stddev);

        geo::Transform2 noise;
        noise.t = geo::Vec2(delta_trans_hat, delta_strafe_hat);
        noise.setRotation(delta_rot_hat);

        sample.pose = sample.pose * movement * noise;
    }
}
