#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/Shape.h>

// Inflation
#include <queue>

// ----------------------------------------------------------------------------------------------------

class CellData
{
public:

  CellData(int index_, int dx_, int dy_, int distance_) :
      index(index_), dx(dx_), dy(dy_), distance(distance_)
  {}

  int index;
  int dx, dy;
  int distance;

};

// ----------------------------------------------------------------------------------------------------

inline bool operator<(const CellData &a, const CellData &b)
{
  return a.distance > b.distance;
}

// ----------------------------------------------------------------------------------------------------

class LineRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    LineRenderResult(std::queue<CellData>& Q, int grid_size, double res) : Q_(Q), grid_size_(grid_size), res_(res),
        p_min(1e9), p_max(-1e9) {}

    void renderLine(const geo::Vector3& p1, const geo::Vector3& p2)
    {
        geo::Vector3 v_diff = p2 - p1;
        double n_diff = v_diff.length();

        geo::Vector3 vd = v_diff / n_diff * res_;
        int n = n_diff / res_ + 1;

        geo::Vector3 p = p1;
        for(int i = 0; i < n; ++i)
        {
            int x = -p.y / res_ + grid_size_ / 2;
            int y = -p.x / res_ + grid_size_ / 2;

            if (x >= 0 && y >= 0 && x < grid_size_ && y <grid_size_) {
                int index = y * grid_size_ + x;
                Q_.push(CellData(index, 0, 0, 0));
            }
            p += vd;
        }

        p_min.x = std::min(p_min.x, std::min(p1.x, p2.x));
        p_max.x = std::max(p_max.x, std::max(p1.x, p2.x));

        p_min.y = std::min(p_min.y, std::min(p1.y, p2.y));
        p_max.y = std::max(p_max.y, std::max(p1.y, p2.y));
    }

private:

    std::queue<CellData>& Q_;
    int grid_size_;
    double res_;

public:

    bool empty;
    geo::Vec2 p_min, p_max;

};

// ----------------------------------------------------------------------------------------------------

void drawLaserPoints(cv::Mat& img, const geo::LaserRangeFinder& lrf, const std::vector<double>& ranges, const cv::Vec3b& clr)
{
    std::vector<geo::Vector3> points;
    if (lrf.rangesToPoints(ranges, points)) {
        for(unsigned int i = 0; i < points.size(); ++i) {
            const geo::Vector3& p = points[i];
            double x = (-p.y * 25) + img.cols / 2;
            double y = (-p.x * 25) + img.rows / 2;

            if (x >= 0 && y >= 0 && x < img.cols && y < img.rows) {
                img.at<cv::Vec3b>(y, x) = clr;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

class KernelCell
{
public:

    KernelCell(int d_index_, int dx_, int dy_) :
        d_index(d_index_), dx(dx_), dy(dy_)
    {}

    int d_index;
    int dx, dy;
};

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() : pose_initialized_(false)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::configure(tue::Configuration config)
{
    std::string laser_topic;
    config.value("laser_topic", laser_topic);

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&LocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    sub_laser_ = nh.subscribe(sub_options);

    a_current_ = 0;
    laser_pose_ = geo::Pose3D::identity();
    laser_pose_.t.z = 0.3;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    last_laser_msg_.reset();
    cb_queue_.callAvailable();

    if (!last_laser_msg_)
        return;

    tue::Timer timer;
    timer.start();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update sensor model
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (lrf_.getNumBeams() != last_laser_msg_->ranges.size())
    {
        lrf_.setNumBeams(last_laser_msg_->ranges.size());
        lrf_.setRangeLimits(last_laser_msg_->range_min, last_laser_msg_->range_max);
        lrf_.setAngleLimits(last_laser_msg_->angle_min, last_laser_msg_->angle_max);
    }

    std::vector<double> sensor_ranges(last_laser_msg_->ranges.size());
    for (unsigned int i = 0; i < last_laser_msg_->ranges.size(); ++i)
        sensor_ranges[i] = last_laser_msg_->ranges[i];

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create world model cross section
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        // if (it->second->id() == "pico_case")
        if (it->second->shape())
            entities.push_back(it->second);
    }

    geo::Pose3D laser_pose;
    laser_pose.t = laser_pose_.t + geo::Vector3(0, 0, 0);
    laser_pose.R.setRPY(0, 0, 0);

    double grid_resolution = 0.025;
    int grid_size = 800;

    std::queue<CellData> Q;
    LineRenderResult render_result(Q, grid_size, grid_resolution);

    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        geo::LaserRangeFinder::RenderOptions options;
        geo::Transform t_inv = laser_pose.inverse() * e->pose();
        options.setMesh(e->shape()->getMesh(), t_inv);
        lrf_.render(options, render_result);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create distance map
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    double max_dist = 0.3;

    cv::Mat distance_map(grid_size, grid_size, CV_32FC1, (max_dist / grid_resolution) * (max_dist / grid_resolution));
    for(int i = 0; i < grid_size; ++i)
    {
        distance_map.at<float>(i, 0) = 0;
        distance_map.at<float>(i, grid_size - 1) = 0;
        distance_map.at<float>(0, i) = 0;
        distance_map.at<float>(grid_size - 1, i) = 0;
    }

    std::vector<KernelCell> kernel;

    kernel.push_back(KernelCell(-distance_map.cols, 0, -1));
    kernel.push_back(KernelCell(distance_map.cols, 0, 1));
    kernel.push_back(KernelCell(-1, -1, 0));
    kernel.push_back(KernelCell( 1,  1, 0));

    while(!Q.empty())
    {
        const CellData& c = Q.front();

        double current_distance = distance_map.at<float>(c.index);
        if (c.distance < current_distance)
        {
            distance_map.at<float>(c.index) = c.distance;
            for(unsigned int i = 0; i < kernel.size(); ++i)
            {
                const KernelCell& kc = kernel[i];
                int new_index = c.index + kc.d_index;
                int dx_new = c.dx + kc.dx;
                int dy_new = c.dy + kc.dy;

                int new_distance = (dx_new * dx_new) + (dy_new * dy_new);
                Q.push(CellData(new_index, dx_new, dy_new, new_distance));
            }
        }

        Q.pop();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create samples
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Pose3D> poses;

    if (!pose_initialized_)
    {
        // Uniform sampling over world

        for(double x = render_result.p_min.x; x < render_result.p_max.x; x += 0.2)
        {
            for(double y = render_result.p_min.y; y < render_result.p_max.y; y += 0.2)
            {
                for(double a = 0; a < 6.28; a += 0.1)
                {
                    geo::Pose3D laser_pose;
                    laser_pose.t = geo::Vector3(x, y, 0);
                    laser_pose.R.setRPY(0, 0, a);

                    poses.push_back(laser_pose);
                }
            }
        }
    }
    else
    {
        poses.push_back(best_laser_pose_);

        for(double dx = -0.3; dx < 0.3; dx += 0.1)
        {
            for(double dy = -0.3; dy < 0.3; dy += 0.1)
            {
                for(double da = -1; da < 1; da += 0.1)
                {
                    geo::Pose3D dT;
                    dT.t = geo::Vector3(dx, dy, 0);
                    dT.R.setRPY(0, 0, da);

                    poses.push_back(best_laser_pose_ * dT);
                }
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Test samples and find sample with lowest error
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Vector3> sensor_points;
    lrf_.rangesToPoints(sensor_ranges, sensor_points);

    int i_step = 1;
    if (poses.size() > 10000)
        i_step = sensor_points.size() / 100;  // limit to 100 beams

    std::cout << "i_step = " << i_step << std::endl;

    double min_sum_sq_error = 1e10;
    for(std::vector<geo::Pose3D>::const_iterator it = poses.begin(); it != poses.end(); ++it)
    {
        const geo::Pose3D& laser_pose = *it;

        double z1 = laser_pose.R.xx / grid_resolution;
        double z2 = laser_pose.R.xy / grid_resolution;
        double z3 = laser_pose.R.yx / grid_resolution;
        double z4 = laser_pose.R.yy / grid_resolution;
        double t1 = laser_pose.t.x / grid_resolution - distance_map.cols / 2;
        double t2 = laser_pose.t.y / grid_resolution - distance_map.cols / 2;

        double sum_sq_error = 0;
        for(unsigned int i = 0; i < sensor_points.size(); i += i_step)
        {
            const geo::Vector3& v = sensor_points[i];
            double py = z3 * v.x + z4 * v.y + t2;
            int mx = -py;

            if (mx > 0 && mx < grid_size)
            {
                double px = z1 * v.x + z2 * v.y + t1;
                int my = -px;

                if (my > 0 && my < grid_size)
                {
                    sum_sq_error += distance_map.at<float>(my, mx);
                }
            }
        }

        if (sum_sq_error < min_sum_sq_error)
        {
            min_sum_sq_error = sum_sq_error;
            best_laser_pose_ = laser_pose;
            pose_initialized_ = true;
        }
    }

    std::cout << min_sum_sq_error << std::endl;
    std::cout << best_laser_pose_ << std::endl;

    std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Visualization
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool visualize = true;
    if (visualize)
    {
        cv::Mat rgb_image(distance_map.rows, distance_map.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        for(int y = 0; y < rgb_image.rows; ++y)
        {
            for(int x = 0; x < rgb_image.cols; ++x)
            {
                int c = distance_map.at<float>(y, x) / ((max_dist / grid_resolution) * (max_dist / grid_resolution)) * 255;
                rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(c, c, c);
            }
        }

        for(unsigned int i = 0; i < sensor_points.size(); ++i)
        {
            const geo::Vector3& p = best_laser_pose_ * sensor_points[i];
            int mx = -p.y / grid_resolution + grid_size / 2;
            int my = -p.x / grid_resolution + grid_size / 2;

            if (mx >= 0 && my >= 0 && mx < grid_size && my <grid_size)
            {
                rgb_image.at<cv::Vec3b>(my, mx) = cv::Vec3b(0, 255, 0);
            }
        }

        // Visualize sensor
        int lmx = -best_laser_pose_.t.y / grid_resolution + grid_size / 2;
        int lmy = -best_laser_pose_.t.x / grid_resolution + grid_size / 2;
        cv::circle(rgb_image, cv::Point(lmx,lmy), 0.3 / grid_resolution, cv::Scalar(0, 0, 255), 1);

        geo::Vector3 d = best_laser_pose_.R * geo::Vector3(0.3, 0, 0);
        int dmx = -d.y / grid_resolution;
        int dmy = -d.x / grid_resolution;
        cv::line(rgb_image, cv::Point(lmx, lmy), cv::Point(lmx + dmx, lmy + dmy), cv::Scalar(0, 0, 255), 1);

        cv::imshow("distance_map", rgb_image);
        cv::waitKey(1);
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    last_laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
