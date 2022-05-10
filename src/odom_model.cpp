#include "ed_localization/odom_model.h"

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double generateRandomGaussian(double sigma)
{
    double x1, x2, w, r;

    do
    {
        do { r = drand48(); } while (r==0.0);
        x1 = 2.0 * r - 1.0;
        do { r = drand48(); } while (r==0.0);
        x2 = 2.0 * r - 1.0;
        w = x1*x1 + x2*x2;
    } while(w > 1.0 || w==0.0);

    return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

namespace ed_localization {

// ----------------------------------------------------------------------------------------------------

OdomModel::OdomModel() : alpha1_(0.2), alpha2_(0.2), alpha3_(0.2), alpha4_(0.2), alpha5_(0.2)
{
}

// ----------------------------------------------------------------------------------------------------

OdomModel::~OdomModel()
{
}

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

void OdomModel::updatePoses(const geo::Transform2& movement, ParticleFilter& pf)
{
    ROS_DEBUG_STREAM_NAMED("odom_model", "Applying a movement of " << movement);
    double delta_trans_sq = movement.t.length2();

    double delta_rot = movement.rotation();
    double delta_rot_sq = delta_rot * delta_rot;

    // Compute noise standard deviations
    double trans_hat_stddev = sqrt(alpha3_ * delta_trans_sq + alpha4_ * delta_rot_sq);
    double rot_hat_stddev = sqrt(alpha1_ * delta_rot_sq + alpha2_ * delta_trans_sq);
    double strafe_hat_stddev = sqrt(alpha4_ * delta_rot_sq + alpha5_ * delta_trans_sq);

    for(std::vector<Sample>::iterator it = pf.samples().begin(); it != pf.samples().end(); ++it)
    {
        Sample& sample = *it;

        // Sample pose differences
        double delta_trans_hat = generateRandomGaussian(trans_hat_stddev);
        double delta_rot_hat = generateRandomGaussian(rot_hat_stddev);
        double delta_strafe_hat = generateRandomGaussian(strafe_hat_stddev);

        geo::Transform2 noise;
        noise.t = geo::Vec2(delta_trans_hat, delta_strafe_hat);
        noise.setRotation(delta_rot_hat);

        sample.pose = sample.pose * movement * noise;
    }
}

}
