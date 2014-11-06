#include "particle_filter.h"

// ----------------------------------------------------------------------------------------------------

ParticleFilter::ParticleFilter() : i_current_(0)
{
}

// ----------------------------------------------------------------------------------------------------

ParticleFilter::~ParticleFilter()
{
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::initUniform(const geo::Vec2& min, const geo::Vec2& max, double t_step, double a_step)
{
    std::vector<Sample>& smpls = samples();

    smpls.clear();
    for(double x = min.x; x < max.x; x += t_step)
        for(double y = min.y; y < max.y; y += t_step)
            for(double a = 0; a < 2 * M_PI; a += a_step)
                smpls.push_back(Sample(geo::Transform2(x, y, a)));

    setUniformWeights();
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::resample(unsigned int num_samples)
{
    std::vector<Sample>& old_samples = samples_[i_current_];
    std::vector<Sample>& new_samples = samples_[1 - i_current_];

    if (old_samples.empty())
        return;

    if (num_samples == 0)
        num_samples = old_samples.size();

    // Build up cumulative probability table for resampling.
    std::vector<double> cum_weights(old_samples.size());
    cum_weights[0] = old_samples[0].weight;
    for(unsigned int i = 1; i < old_samples.size(); ++i)
        cum_weights[i] = cum_weights[i - 1] + old_samples[i].weight;

    new_samples.resize(num_samples);
    for(std::vector<Sample>::iterator it = new_samples.begin(); it != new_samples.end(); ++it)
    {
        Sample& new_sample = *it;

        double r = drand48();
        unsigned int i_sample = 0;
        for(i_sample = 0; i_sample < old_samples.size(); ++i_sample)
        {
            if (r < cum_weights[i_sample])
                break;
        }

        new_sample = old_samples[i_sample];
    }

    i_current_ = 1 - i_current_;
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::normalize()
{
    std::vector<Sample>& smpls = samples();

    double total_weight = 0;
    for(std::vector<Sample>::iterator it = smpls.begin(); it != smpls.end(); ++it)
        total_weight += it->weight;

    if (total_weight > 0)
    {
        for(std::vector<Sample>::iterator it = smpls.begin(); it != smpls.end(); ++it)
            it->weight /= total_weight;
    }
    else
    {
        setUniformWeights();
    }
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::setUniformWeights()
{
    double uni_weight = 1.0 / samples().size();
    for(std::vector<Sample>::iterator it = samples().begin(); it != samples().end(); ++it)
        it->weight = uni_weight;
}

