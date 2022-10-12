#include "ed_localization/particle_filter.h"

#include <boost/filesystem.hpp>

#include <ros/console.h>

#include <tue/filesystem/path.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <time.h>

namespace ed_localization {

/**
 * @brief Get current date/time, format is YYYY-MM-DD-HH-mm-ss
 * @return
 */
std::string currentDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);

    // Visit https://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

    return buf;
}

// ----------------------------------------------------------------------------------------------------

ParticleFilter::ParticleFilter() :
    min_samples_(0),
    max_samples_(0),
    kld_err_(0),
    kld_z_(0),
    alpha_slow_(0),
    alpha_fast_(0),
    w_slow_(0),
    w_fast_(0),
    i_current_(0),
    kd_tree_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

void ParticleFilter::configure(tue::Configuration config)
{
    // Extra variable needed as tue::Configuration doesn't support
    // unisgned interters
    int min = 0, max = 0;
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

    std::array<double, 3> cell_size = {0.5, 0.5, 10*M_PI/180};
    config.value("cell_size_x", cell_size[0], tue::config::OPTIONAL);
    config.value("cell_size_y", cell_size[1], tue::config::OPTIONAL);
    config.value("cell_size_theta", cell_size[2], tue::config::OPTIONAL);

    samples_[0].reserve(max_samples_);
    samples_[1].reserve(max_samples_);

    cluster_cache_.reserve(max_samples_);

    limit_cache_.clear();
    limit_cache_.resize(max_samples_, 0);

    kd_tree_ = std::make_unique<ed_localization::KDTree>(max_samples_, cell_size);

    ROS_INFO_STREAM_NAMED("pf", "min_samples: " << min_samples_ << ", max_samples: " << max_samples_ << std::endl
                          << "kld_err: " << kld_err_ << ", kld_z: " << kld_z_ << std::endl
                          << "recovery_alpha_slow: " << alpha_slow_ << ", recovery_alpha_fast: " << alpha_fast_ << std::endl
                          << "cell_size_x: " << cell_size[0] << ", cell_size_y: " << cell_size[1] << ", cell_size_theta: " << cell_size[2]);
}

// ----------------------------------------------------------------------------------------------------

ParticleFilter::~ParticleFilter()
{
}

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

    for(double x = min.x; x < max.x; x += step_x)
        for(double y = min.y; y < max.y; y += step_y)
            for(double a = a_min; a < a_max; a += step_yaw)
                smpls.push_back(Sample(geo::Transform2(x, y, a)));

    setUniformWeights();

    kd_tree_->clear();
    for (const Sample& sample : samples())
        kd_tree_->insert(sample.pose, sample.weight);
}

// ----------------------------------------------------------------------------------------------------

bool compareSamples(const Sample& a, const Sample& b)
{
    return a.weight > b.weight;
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
    for (uint i=0; i<old_samples.size(); ++i)
        c[i+1] = c[i]+ old_samples[i].weight;

    double w_diff = 1 - w_fast_ / w_slow_;
    if(w_diff < 0)
        w_diff = 0;

    // Create the kd tree for adaptive sampling;
    kd_tree_->clear();

    // Draw samples from set a to create set b.
    new_samples.clear();

    while(new_samples.size() < max_samples_)
    {
        new_samples.push_back(Sample());
        Sample& new_sample = new_samples.back();

        double r = drand48();

        if(r < w_diff)
            new_sample.pose = gen_random_pose_function();
        else
        {
            // Naive discrete event sampler
            uint i=0;
            for(; i<old_samples.size(); ++i)
            {
                if((c[i] <= r) && (r < c[i+1]))
                    break;
            }

            // Add sample to list
            new_sample.pose =  old_samples[i].pose;
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
    double x = 1 - b + c; // x = a - b + c

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
    for(const Cluster& cluster : clstrs)
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
    for(std::vector<Sample>::iterator it = smpls.begin(); it != smpls.end(); ++it)
        total_weight += it->weight;

    double w_avg = total_weight / samples().size();

    if (total_weight > 0)
    {
        if (update_filter)
        {
            // slow
            if(w_slow_ == 0)
                w_slow_ = w_avg;
            else
                w_slow_ += alpha_slow_ * (w_avg - w_slow_);

            // Fast
            if(w_fast_ == 0)
              w_fast_ = w_avg;
            else
              w_fast_ += alpha_fast_ * (w_avg - w_fast_);
        }

        for(std::vector<Sample>::iterator it = smpls.begin(); it != smpls.end(); ++it)
            it->weight /= total_weight;
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
    std::array<double, 4> m{ {0, 0, 0, 0} };
    std::array<std::array<double, 2>, 2> c{ {{0, 0}, {0, 0}} };

    // Compute cluster stats
    for (const Sample& sample : samples())
    {
        // Get the cluster label for this sample
        int cidx = kd_tree_->getCluster(sample.pose);
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
        for (unsigned int j=0; j<2; ++j)
        {
            for (unsigned int k=0; k<2; ++k)
            {
                cluster.c[j][k] += sample.weight * sample.pose.t.m[j] * sample.pose.t.m[k];
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
        for (unsigned int j=0; j<2; ++j)
            for (unsigned int k=0; k<2; ++k)
                cluster.cov.m[j * 3 + k] = cluster.c[j][k] / cluster.weight - cluster.mean.t.m[j] * cluster.mean.t.m[k];

        // Covariance in angular components
        cluster.cov.m[8] = -2 * log(sqrt(cluster.m[2] * cluster.m[2] + cluster.m[3] * cluster.m[3]));

    }

    // Compute overall filter stats
    mean_cache_.t.x = m[0] / weight;
    mean_cache_.t.y = m[1] / weight;
    mean_cache_.setRotation(atan2(m[3], m[2]));

    // Covariance in linear components
    for (unsigned int j=0; j<2; ++j)
        for (unsigned int k=0; k<2; ++k)
            cov_cache_.m[j * 3 + k] = c[j][k] / weight - mean_cache_.t.m[j] * mean_cache_.t.m[k];

    // Covariance in angular components
    cov_cache_.m[8] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
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
    double uni_weight = 1.0 / samples().size();
    for(std::vector<Sample>::iterator it = samples().begin(); it != samples().end(); ++it)
        it->weight = uni_weight;
}

// ----------------------------------------------------------------------------------------------------

bool ParticleFilter::writeCSV(std::string file_name)
{
    const static std::string current_date_time = currentDateTime();
    const static std::string home_dir = getenv("HOME");
    const static std::string file_dir = tue::filesystem::Path(home_dir).join("ros").join("data").join("ed_localization").join(current_date_time).string();

    if (!boost::filesystem::create_directories(file_dir) && !boost::filesystem::is_directory(file_dir))
    {
        ROS_ERROR_STREAM_NAMED("pf", "Could not create directory: '" << file_dir << "'");
        return false;
    }

    if (file_name.size() < 4 || file_name.substr(file_name.size()-4, 4) != ".csv")
        file_name.append(".csv");

    const std::string file_path = tue::filesystem::Path(file_dir).join(file_name).string();

    std::ofstream file(file_path);
    file << "x,y,theta,weight" << std::endl;

    try
    {
        for (const auto& sample : samples())
        {
            file << sample.pose.t.x << "," << sample.pose.t.y << "," << sample.pose.rotation() << "," << sample.weight << std::endl;
        }
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED("pf", "Could not write all the samples to file '" << file_path << "': " << ex.what());
        file.close();
        return false;
    }

    file.close();

    return true;
}

}
