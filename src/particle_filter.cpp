#include "particle_filter.h"

#include <algorithm>
#include <cmath>

#include <ros/console.h>

// ----------------------------------------------------------------------------------------------------

ParticleFilter::ParticleFilter() : i_current_(0), min_samples_(0), max_samples_(0), kld_err_(0), kld_z_(0), kd_tree_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::configure(tue::Configuration config)
{
    // Extra variable needed as tue::Configuration doesn't support
    // unisgned interters
    int min, max;
    config.value("min_particles", min);
    config.value("max_particles", max);
    min_samples_ = min;
    max_samples_ = max;

    samples_[0].reserve(max_samples_);
    samples_[1].reserve(max_samples_);

    kld_err_ = 0.01;
    kld_z_ = 0.99;
    config.value("kld_err", kld_err_, tue::config::OPTIONAL);
    config.value("kld_z", kld_z_, tue::config::OPTIONAL);

    limit_cache_.clear();
    limit_cache_.resize(max_samples_, 0);

    std::array<double, 3> cell_size = {0.5, 0.5, 10*M_PI/180};
    config.value("cell_size_x", cell_size[0], tue::config::OPTIONAL);
    config.value("cell_size_y", cell_size[1], tue::config::OPTIONAL);
    config.value("cell_size_theta", cell_size[2], tue::config::OPTIONAL);

    kd_tree_.reset(new KDTree(max_samples_, cell_size));

    ROS_INFO_STREAM("[ED Localization] min_samples: " << min_samples_ << ", max_samples: " << max_samples_);
    ROS_INFO_STREAM("[ED Localization] kld_err: " << kld_err_ << ", kld_z: " << kld_z_);
    ROS_INFO_STREAM("[ED Localization] cell_size_x: " << cell_size[0] << ", cell_size_y: " << cell_size[1] << ", cell_size_theta: " << cell_size[2]);
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

void ParticleFilter::resample()
{
    std::vector<Sample>& old_samples = samples_[i_current_];
    std::vector<Sample>& new_samples = samples_[1 - i_current_];

    if (old_samples.empty())
        return;

    // Build up cumulative probability table for resampling.
    // TODO: Replace this with a more efficient procedure
    // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
    std::vector<double> c;
    c.resize(old_samples.size() + 1, 0);
    for (uint i=0; i<old_samples.size(); ++i)
        c[i+1] = c[i]+ old_samples[i].weight;

    // Create the kd tree for adaptive sampling;
    kd_tree_->clear();

    // Draw samples from set a to create set b.
    new_samples.clear();

    while(new_samples.size() < max_samples_)
    {
        new_samples.push_back(Sample());
        Sample& new_sample = new_samples.back();

        // Naive discrete event sampler
        double r = drand48();
        uint i=0;
        for(; i<old_samples.size(); ++i)
        {
            if((c[i] <= r) && (r < c[i+1]))
                break;
        }

        Sample& old_sample = old_samples[i];

        // Add sample to list
        new_sample.pose = old_sample.pose;

        new_sample.weight = 1;

        // Add sample to histogram
        kd_tree_->insert(new_sample.pose, new_sample.weight);

        // See if we have enough samples yet
        if (new_samples.size() >= resampleLimit(kd_tree_->getLeafCount()))
            break;
    }

    i_current_ = 1 - i_current_;

    normalize();
}

// ----------------------------------------------------------------------------------------------------

unsigned int ParticleFilter::resampleLimit(unsigned int k)
{
    if (limit_cache_[k-1] != 0)
        return limit_cache_[k-1];

    if (k <= 1)
    {
        limit_cache_[k-1] = max_samples_;
        return max_samples_;
    }

    // double a = 1;
    double b = 2 / (9 * (static_cast<double>(k - 1)));
    double c = sqrt(2 / (9 * (static_cast<double>(k - 1)))) * kld_z_;
    double x = 1 - b + c;

    unsigned int n = std::ceil((k - 1) / (2 * kld_err_) * x * x * x);

    if (n < min_samples_)
    {
        limit_cache_[k-1] = min_samples_;
        return min_samples_;
    }
    if (n > max_samples_)
    {
        limit_cache_[k-1] = min_samples_;
        return max_samples_;
    }

    limit_cache_[k-1] = n;
    return n;
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

