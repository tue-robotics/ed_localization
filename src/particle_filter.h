#ifndef ED_LOCALIZATION_PARTICLE_FILTER_H_
#define ED_LOCALIZATION_PARTICLE_FILTER_H_

#include "kdtree.h"
#include "transform.h"

#include <geolib/datatypes.h>

#include <tue/config/configuration.h>

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

    void configure(tue::Configuration config);

    void initUniform(const geo::Vec2& min, const geo::Vec2& max, double t_step,
                     double a_min, double a_max, double a_step);

    void resample(std::function<geo::Transform2()> gen_random_pose_function);

    unsigned int resampleLimit(unsigned int number_bins);

    std::vector<Sample>& samples() { return samples_[i_current_]; }

    const std::vector<Sample>& samples() const { return samples_[i_current_]; }

    const Sample& bestSample() const;

    geo::Transform2 calculateMeanPose() const;

    void normalize(bool update_filter=false);

private:

    unsigned int min_samples_, max_samples_;
    double kld_err_, kld_z_;
    double alpha_slow_, alpha_fast_;

    /**
     * @brief Current running averages of likelihood of samples
     */
    double w_slow_, w_fast_;

    int i_current_;
    std::vector<Sample> samples_[2];

    std::unique_ptr<KDTree> kd_tree_;

    std::vector<unsigned int> limit_cache_;

    void setUniformWeights();

};

#endif
