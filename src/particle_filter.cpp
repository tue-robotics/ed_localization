#include "particle_filter.h"

#include "kdtree.h"

#include <geolib/math_types.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tue/config/configuration.h>
#include <tue/config/types.h>

#include <algorithm>
#include <array>
#include <cmath>
// drand48() is POSIX, declared by <stdlib.h>; <cstdlib> only guarantees the ISO C subset.
#include <functional>
#include <memory>
#include <numbers>
#include <stdlib.h> // NOLINT(modernize-deprecated-headers)
#include <vector>

// ----------------------------------------------------------------------------------------------------

ParticleFilter::ParticleFilter() :
    min_samples_(0), max_samples_(0), kld_err_(0), kld_z_(0), alpha_slow_(0), alpha_fast_(0), w_slow_(0), w_fast_(0),
    i_current_(0), kd_tree_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::configure(tue::Configuration config)
{
    // Extra variable needed as tue::Configuration doesn't support
    // unisgned interters
    int min = 0;
    int max = 0;
    config.value("min_particles", min);
    config.value("max_particles", max);
    min_samples_ = min;
    max_samples_ = max;

    kld_err_ = 0.01;
    kld_z_ = 0.99;
    config.value("kld_err", kld_err_, tue::config::OPTIONAL);
    config.value("kld_z", kld_z_, tue::config::OPTIONAL);

    alpha_slow_ = 0;
    alpha_fast_ = 0;
    config.value("recovery_alpha_slow", alpha_slow_, tue::config::OPTIONAL);
    config.value("recovery_alpha_fast", alpha_fast_, tue::config::OPTIONAL);

    std::array<double, 3> cell_size = {0.5, 0.5, 10 * std::numbers::pi / 180};
    config.value("cell_size_x", cell_size[0], tue::config::OPTIONAL);
    config.value("cell_size_y", cell_size[1], tue::config::OPTIONAL);
    config.value("cell_size_theta", cell_size[2], tue::config::OPTIONAL);

    samples_[0].reserve(max_samples_);
    samples_[1].reserve(max_samples_);

    cluster_cache_.reserve(max_samples_);

    limit_cache_.clear();
    limit_cache_.resize(max_samples_, 0);

    kd_tree_ = std::make_unique<KDTree>(max_samples_, cell_size);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("Localization"),
                       "min_samples: " << min_samples_ << ", max_samples: " << max_samples_ << '\n'
                                       << "kld_err: " << kld_err_ << ", kld_z: " << kld_z_ << '\n'
                                       << "recovery_alpha_slow: " << alpha_slow_
                                       << ", recovery_alpha_fast: " << alpha_fast_ << '\n'
                                       << "cell_size_x: " << cell_size[0] << ", cell_size_y: " << cell_size[1]
                                       << ", cell_size_theta: " << cell_size[2]);
}

// ----------------------------------------------------------------------------------------------------

ParticleFilter::~ParticleFilter() = default;

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::initUniform(const geo::Vec2& min, const geo::Vec2& max, double a_min, double a_max)
{
    clearCache();

    std::vector<Sample>& smpls = samples();

    smpls.clear();

    const double range_x = max.x - min.x;
    const double range_y = max.y - min.y;
    const double range_yaw = a_max - a_min;

    const double cbrt_samples = ceil(std::cbrt(max_samples_));
    const double step_x = range_x / cbrt_samples;
    const double step_y = range_y / cbrt_samples;
    const double step_yaw = range_yaw / cbrt_samples;

    // Walking the grid with floating point counters accumulates rounding error, which here only
    // shifts the sample count by at most one per axis - acceptable for seeding a particle filter.
    // NOLINTBEGIN(clang-analyzer-security.FloatLoopCounter)
    for (double x = min.x; x < max.x; x += step_x)
        for (double y = min.y; y < max.y; y += step_y)
            for (double a = a_min; a < a_max; a += step_yaw)
                smpls.emplace_back(geo::Transform2(x, y, a));
    // NOLINTEND(clang-analyzer-security.FloatLoopCounter)

    setUniformWeights();

    kd_tree_->clear();
    for (const Sample& sample : samples())
        kd_tree_->insert(sample.pose, sample.weight);
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::resample(const std::function<geo::Transform2()>& gen_random_pose_function)
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
    for (unsigned int i = 0; i < old_samples.size(); ++i)
        c[i + 1] = c[i] + old_samples[i].weight;

    double w_diff = 1 - (w_fast_ / w_slow_);
    w_diff = std::max<double>(w_diff, 0);

    // Create the kd tree for adaptive sampling;
    kd_tree_->clear();

    // Draw samples from set a to create set b.
    new_samples.clear();

    while (new_samples.size() < max_samples_)
    {
        new_samples.emplace_back();
        Sample& new_sample = new_samples.back();

        double const r = drand48();

        if (r < w_diff)
            new_sample.pose = gen_random_pose_function();
        else
        {
            // Naive discrete event sampler
            unsigned int i = 0;
            for (; i < old_samples.size(); ++i)
            {
                if ((c[i] <= r) && (r < c[i + 1]))
                    break;
            }

            // Add sample to list
            new_sample.pose = old_samples[i].pose;
        }

        new_sample.weight = 1;

        // Add sample to histogram
        kd_tree_->insert(new_sample.pose, new_sample.weight);

        // See if we have enough samples yet
        if (new_samples.size() >= resampleLimit(kd_tree_->getLeafCount()))
            break;
    }

    switchSamples();

    normalize();
}

// ----------------------------------------------------------------------------------------------------

unsigned int ParticleFilter::resampleLimit(unsigned int k)
{
    if (limit_cache_[k - 1] != 0)
        return limit_cache_[k - 1];

    if (k <= 1)
    {
        limit_cache_[k - 1] = max_samples_;
        return max_samples_;
    }

    // double a = 1;
    double const b = 2 / (9 * (static_cast<double>(k - 1)));
    double const c = sqrt(2 / (9 * (static_cast<double>(k - 1)))) * kld_z_;
    double const x = 1 - b + c; // x = a - b + c

    unsigned int const n = std::ceil((k - 1) / (2 * kld_err_) * x * x * x);

    if (n < min_samples_)
    {
        limit_cache_[k - 1] = min_samples_;
        return min_samples_;
    }
    if (n > max_samples_)
    {
        limit_cache_[k - 1] = min_samples_;
        return max_samples_;
    }

    limit_cache_[k - 1] = n;
    return n;
}

// ----------------------------------------------------------------------------------------------------

const Sample& ParticleFilter::bestSample() const
{
    const std::vector<Sample>& smpls = samples();

    const Sample* best_sample = &smpls.front();
    for (const auto& s : smpls)
    {
        if (s.weight > best_sample->weight)
            best_sample = &s;
    }

    return *best_sample;
}

// ----------------------------------------------------------------------------------------------------

const std::vector<Cluster>& ParticleFilter::clusters() const
{
    if (cluster_cache_.empty())
        computeClusterStats();

    return cluster_cache_;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 ParticleFilter::calculateMeanPose() const
{
    const std::vector<Cluster>& clstrs = clusters();

    geo::Transform2 mean(0, 0, 0);

    double max_weight = 0;
    for (const Cluster& cluster : clstrs)
    {
        if (cluster.weight > max_weight)
        {
            mean = cluster.mean;
            max_weight = cluster.weight;
        }
    }

    return mean;
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::normalize(bool update_filter)
{
    std::vector<Sample>& smpls = samples();

    double total_weight = 0;
    for (auto& smpl : smpls)
        total_weight += smpl.weight;

    double const w_avg = total_weight / static_cast<double>(samples().size());

    if (total_weight > 0)
    {
        if (update_filter)
        {
            // slow
            if (w_slow_ == 0)
                w_slow_ = w_avg;
            else
                w_slow_ += alpha_slow_ * (w_avg - w_slow_);

            // Fast
            if (w_fast_ == 0)
                w_fast_ = w_avg;
            else
                w_fast_ += alpha_fast_ * (w_avg - w_fast_);
        }

        for (auto& smpl : smpls)
            smpl.weight /= total_weight;
    }
    else
    {
        setUniformWeights();
    }
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::computeClusterStats() const
{
    // Cluster the samples
    kd_tree_->cluster();

    // Initialize overall filter stats
    double weight = 0;

    // Workspace
    std::array<double, 4> m{{0, 0, 0, 0}};
    std::array<std::array<double, 2>, 2> c{{{0, 0}, {0, 0}}};

    // Compute cluster stats
    for (const Sample& sample : samples())
    {
        // Get the cluster label for this sample
        int const cidx = kd_tree_->getCluster(sample.pose);
        if (cidx < 0)
            continue;

        if (static_cast<unsigned int>(cidx + 1) > cluster_cache_.size())
            cluster_cache_.resize(cidx + 1);

        Cluster& cluster = cluster_cache_[cidx];

        cluster.count += 1;
        cluster.weight += sample.weight;
        weight += sample.weight;

        // Compute mean
        cluster.m[0] += sample.weight * sample.pose.t.x;
        cluster.m[1] += sample.weight * sample.pose.t.y;
        cluster.m[2] += sample.weight * cos(sample.pose.rotation());
        cluster.m[3] += sample.weight * sin(sample.pose.rotation());

        m[0] += cluster.m[0];
        m[1] += cluster.m[1];
        m[2] += cluster.m[2];
        m[3] += cluster.m[3];

        // Compute covariance in linear components
        for (unsigned int j = 0; j < 2; ++j)
        {
            for (unsigned int k = 0; k < 2; ++k)
            {
                cluster.c[j][k] += sample.weight * sample.pose.t[j] * sample.pose.t[k];
                c[j][k] += cluster.c[j][k];
            }
        }
    }

    // Normalize
    for (Cluster& cluster : cluster_cache_)
    {
        cluster.mean.t.x = cluster.m[0] / cluster.weight;
        cluster.mean.t.y = cluster.m[1] / cluster.weight;
        cluster.mean.setRotation(atan2(cluster.m[3], cluster.m[2]));

        // Covariance in linear components
        for (unsigned int j = 0; j < 2; ++j)
            for (unsigned int k = 0; k < 2; ++k)
                cluster.cov[(j * 3) + k] = (cluster.c[j][k] / cluster.weight) - (cluster.mean.t[j] * cluster.mean.t[k]);

        // Covariance in angular components
        cluster.cov[8] = -2 * log(sqrt((cluster.m[2] * cluster.m[2]) + (cluster.m[3] * cluster.m[3])));
    }

    // Compute overall filter stats
    mean_cache_.t.x = m[0] / weight;
    mean_cache_.t.y = m[1] / weight;
    mean_cache_.setRotation(atan2(m[3], m[2]));

    // Covariance in linear components
    for (unsigned int j = 0; j < 2; ++j)
        for (unsigned int k = 0; k < 2; ++k)
            cov_cache_[(j * 3) + k] = (c[j][k] / weight) - (mean_cache_.t[j] * mean_cache_.t[k]);

    // Covariance in angular components
    cov_cache_[8] = -2 * log(sqrt((m[2] * m[2]) + (m[3] * m[3])));
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::clearCache() const
{
    cluster_cache_.clear();
    mean_cache_ = geo::Transform2::identity();
    cov_cache_ = geo::Mat3::identity();
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::switchSamples()
{
    clearCache();
    i_current_ = 1 - i_current_;
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::setUniformWeights()
{
    double const uni_weight = 1.0 / static_cast<double>(samples().size());
    for (auto& it : samples())
        it.weight = uni_weight;
}
