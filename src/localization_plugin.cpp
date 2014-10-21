#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

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

    double best_score = -1e10;
    std::vector<double> best_ranges;
    geo::Pose3D best_pose;
    double a_best;

    int k = 0;
    for(double x = -0.2; x < 0.2; x += 0.1)
    {
        for(double y = -0.2; y < 0.2; y += 0.1)
        {
            for(double a = -0.2; a < 0.2; a += 0.1)
            {
                ++k;

                tue::Timer timer;
                timer.start();

                geo::Pose3D laser_pose;
                laser_pose.t = laser_pose_.t + geo::Vector3(x, y, 0);
                laser_pose.R.setRPY(0, 0, a_current_ + a);

                std::vector<double> ranges_out;
                for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
                {
                    const ed::EntityConstPtr& e = *it;
                    lrf_.render(*e->shape(), laser_pose, e->pose(), ranges_out);
                }

                double score = 0;
                for(unsigned int i = 0; i < ranges_in.size(); ++i)
                {
                    double diff = std::abs(ranges_in[i] - ranges_out[i]);
                    diff = std::min(diff, 0.3);
                    score -= (diff * diff);
                }

                if (score > best_score)
                {
                    best_score = score;
                    best_ranges = ranges_out;
                    best_pose = laser_pose;
                    a_best = a_current_ + a;
                }

//                std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

            }
        }
    }

    std::cout << k << " samples" << std::endl;

    laser_pose_ = best_pose;
    a_current_ = a_best;

    // Visualization
    cv::Mat img(600, 600, CV_8UC3, cv::Scalar(30, 30, 30));
    drawLaserPoints(img, lrf_, ranges_in, cv::Vec3b(255, 255, 255));
    drawLaserPoints(img, lrf_, best_ranges, cv::Vec3b(0, 255, 0));

    cv::imshow("laser", img);
    cv::waitKey(3);


}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    last_laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
