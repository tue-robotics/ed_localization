#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

// Inflation
#include <queue>

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

class CellData
{
public:

  CellData(int index_, int dx_, int dy_, double distance_) :
      index(index_), dx(dx_), dy(dy_), distance(distance_)
  {}

  int index;
  int dx, dy;
  double distance;

};

// ----------------------------------------------------------------------------------------------------

inline bool operator<(const CellData &a, const CellData &b)
{
  return a.distance > b.distance;
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

LocalizationPlugin::LocalizationPlugin()
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

    if (lrf_.getNumBeams() != last_laser_msg_->ranges.size())
    {
        lrf_.setNumBeams(last_laser_msg_->ranges.size());
        lrf_.setRangeLimits(last_laser_msg_->range_min, last_laser_msg_->range_max);
        lrf_.setAngleLimits(last_laser_msg_->angle_min, last_laser_msg_->angle_max);
    }

    std::vector<double> ranges_in(last_laser_msg_->ranges.size());
    for (unsigned int i = 0; i < last_laser_msg_->ranges.size(); ++i)
        ranges_in[i] = last_laser_msg_->ranges[i];

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        if (it->second->shape())
            entities.push_back(it->second);
    }

    geo::Pose3D laser_pose;
    laser_pose.t = laser_pose_.t + geo::Vector3(0, 0, 0);
    laser_pose.R.setRPY(0, 0, 0);

    std::vector<double> ranges_out;
    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        lrf_.render(*e->shape(), laser_pose, e->pose(), ranges_out);
    }

    double max_dist = 0.3;

    tue::Timer timer;
    timer.start();

    double res = 0.025;

    cv::Mat distance_map(800, 800, CV_32FC1, (max_dist / res) * (max_dist / res));

    std::vector<geo::Vector3> points;
    lrf_.rangesToPoints(ranges_out, points);

    std::queue<CellData> Q;
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        const geo::Vector3& p = points[i];
        int x = -p.y / res + distance_map.cols / 2;
        int y = -p.x / res + distance_map.rows / 2;

        if (x >= 0 && y >= 0 && x < distance_map.cols && y < distance_map.rows) {
            int index = y * distance_map.cols + x;
            Q.push(CellData(index, 0, 0, 0));
        }
    }

    std::vector<KernelCell> kernel;

    kernel.push_back(KernelCell(-distance_map.cols, 0, -1));
    kernel.push_back(KernelCell(distance_map.cols, 0, 1));
    kernel.push_back(KernelCell(-1, -1, 0));
    kernel.push_back(KernelCell( 1,  1, 0));

    while(!Q.empty())
    {
        const CellData& c = Q.front();
        int dx = c.dx;
        int dy = c.dy;
        int index = c.index;
        double distance = c.distance;
        Q.pop();

        double current_distance = distance_map.at<float>(index);
        if (distance < current_distance)
        {
            distance_map.at<float>(index) = distance;
            for(unsigned int i = 0; i < kernel.size(); ++i)
            {
                const KernelCell& kc = kernel[i];
                int new_index = index + kc.d_index;
                int dx_new = dx + kc.dx;
                int dy_new = dy + kc.dy;

//                double new_distance = distance + kc.ddistance;
                double new_distance = (dx_new * dx_new) + (dy_new * dy_new);
                Q.push(CellData(new_index, dx_new, dy_new, new_distance));
            }
        }
    }

    std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

    cv::imshow("distance_map", distance_map / ((max_dist / res) * (max_dist / res)));
    cv::waitKey(1);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    last_laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
