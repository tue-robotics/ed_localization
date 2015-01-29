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

void ParticleFilter::initUniform(const geo::Vec2& min, const geo::Vec2& max, double t_step,
                                 double a_min, double a_max, double a_step)
{
    std::vector<Sample>& smpls = samples();

    smpls.clear();
    for(double x = min.x; x < max.x; x += t_step)
        for(double y = min.y; y < max.y; y += t_step)
            for(double a = a_min; a < a_max; a += a_step)
                smpls.push_back(Sample(geo::Transform2(x, y, a)));

    setUniformWeights();
}

// ----------------------------------------------------------------------------------------------------

bool compareSamples(const Sample& a, const Sample& b)
{
    return a.weight > b.weight;
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

    // Sort all samples (decreasing weight)
    std::sort(old_samples.begin(), old_samples.end(), compareSamples);

    int k = 0;
    new_samples.resize(num_samples);
    for(std::vector<Sample>::const_iterator it = old_samples.begin(); it != old_samples.end(); ++it)
    {
        const Sample& old_sample = *it;

        int l = std::min<int>(k + 1 + old_sample.weight * num_samples, num_samples - 1);

        for(int i = k; i <= l; ++i)
            new_samples[i] = old_sample;

        k = l + 1;

        if (k >= num_samples)
            break;
    }

//    // Build up cumulative probability table for resampling.
//    std::vector<double> cum_weights(old_samples.size());
//    cum_weights[0] = old_samples[0].weight;
//    for(unsigned int i = 1; i < old_samples.size(); ++i)
//        cum_weights[i] = cum_weights[i - 1] + old_samples[i].weight;

//    new_samples.resize(num_samples);
//    for(std::vector<Sample>::iterator it = new_samples.begin(); it != new_samples.end(); ++it)
//    {
//        Sample& new_sample = *it;

//        double r = ((double) rand() / (RAND_MAX));

//        unsigned int i_sample = 0;
//        for(i_sample = 0; i_sample < old_samples.size(); ++i_sample)
//        {
//            if (r < cum_weights[i_sample])
//                break;
//        }

//        new_sample = old_samples[i_sample];
//    }

//    new_samples[0] = best_sample;

    i_current_ = 1 - i_current_;

    normalize();
}

// ----------------------------------------------------------------------------------------------------

const Sample& ParticleFilter::bestSample() const
{
    const std::vector<Sample>& smpls = samples();

    const Sample* best_sample = &smpls.front();
    for(std::vector<Sample>::const_iterator it = smpls.begin(); it != smpls.end(); ++it)
    {
        const Sample& s = *it;
        if (s.weight > best_sample->weight)
            best_sample = &s;
    }

    return *best_sample;
}

// ----------------------------------------------------------------------------------------------------

// TODO: deal with particle clusters (taking the average of multiple clusters of particles does not make sense)
geo::Transform2 ParticleFilter::calculateMeanPose() const
{
    const std::vector<Sample>& smpls = samples();

    geo::Transform2 mean;
    mean.t = geo::Vec2(0, 0);
    geo::Vec2 rot_v(0, 0);

    for(std::vector<Sample>::const_iterator it = smpls.begin(); it != smpls.end(); ++it)
    {
        const Sample& s = *it;
        mean.t += s.weight * s.pose.matrix().t;
        rot_v.x += s.weight * s.pose.matrix().R.xx;
        rot_v.y += s.weight * s.pose.matrix().R.yx;
    }

    rot_v.normalize();
    mean.R.xx = rot_v.x;
    mean.R.yx = rot_v.y;
    mean.R.xy = -mean.R.yx;
    mean.R.yy = mean.R.xx;

    return mean;
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

