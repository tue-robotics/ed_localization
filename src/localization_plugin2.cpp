#include "localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

class LineRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    LineRenderResult(std::vector<geo::Vec3>& lines_start, std::vector<geo::Vec3>& lines_end)
        : lines_start_(lines_start), lines_end_(lines_end), p_min(1e9), p_max(-1e9) {}

    void renderLine(const geo::Vector3& p1, const geo::Vector3& p2)
    {
        lines_start_.push_back(p1);
        lines_end_.push_back(p2);

        p_min.x = std::min(p_min.x, std::min(p1.x, p2.x));
        p_max.x = std::max(p_max.x, std::max(p1.x, p2.x));

        p_min.y = std::min(p_min.y, std::min(p1.y, p2.y));
        p_max.y = std::max(p_max.y, std::max(p1.y, p2.y));
    }

private:

    std::vector<geo::Vec3>& lines_start_;
    std::vector<geo::Vec3>& lines_end_;

public:

    geo::Vec2 p_min, p_max;

};

// ----------------------------------------------------------------------------------------------------

void renderLine(const geo::LaserRangeFinder& lrf, const geo::Vector3& p1, const geo::Vector3& p2, std::vector<double>& ranges)
{
    double a1 = lrf.getAngle(p1.getX(), p1.getY());
    double a2 = lrf.getAngle(p2.getX(), p2.getY());

    double a_min = std::min(a1, a2);
    double a_max = std::max(a1, a2);

    int i_min = lrf.getAngleUpperIndex(a_min);
    int i_max = lrf.getAngleUpperIndex(a_max);

    geo::Vector3 s = p2 - p1;

    // d = (q1 - ray_start) x s / (r x s)
    //   = (q1 x s) / (r x s)

    if (a_max - a_min < M_PI)
    {
        // line is in front of sensor
        for(int i = i_min; i < i_max; ++i)
        {
            const geo::Vector3& r = lrf.rayDirections()[i];
            double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);
            if (d > 0 && (ranges[i] == 0 || d < ranges[i]))
                ranges[i] = d;
        }
    }
    else
    {
        // line is behind sensor
        for(int i = 0; i < i_min; ++i)
        {
            const geo::Vector3& r = lrf.rayDirections()[i];
            double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);
            if (d > 0 && (ranges[i] == 0 || d < ranges[i]))
                ranges[i] = d;
        }

        for(int i = i_max; i < lrf.rayDirections().size(); ++i)
        {
            const geo::Vector3& r = lrf.rayDirections()[i];
            double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);
            if (d > 0 && (ranges[i] == 0 || d < ranges[i]))
                ranges[i] = d;
        }
    }
}

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
    // -     Create samples
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Pose3D> poses;

    if (!pose_initialized_)
    {
        // Uniform sampling over world

        for(double x = -5; x < 5; x += 0.2)
        {
            for(double y = -5; y < 5; y += 0.2)
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

    unsigned int num_beams = last_laser_msg_->ranges.size();
//    if (poses.size() > 10000)
        num_beams = 100;  // limit to 100 beams

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update sensor model
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    int i_step = last_laser_msg_->ranges.size() / num_beams;
    std::vector<double> sensor_ranges;
    for (unsigned int i = 0; i < last_laser_msg_->ranges.size(); i += i_step)
    {
        double r = last_laser_msg_->ranges[i];

        // Check for Inf
        if (r != r || r > last_laser_msg_->range_max)
            r = 0;
        sensor_ranges.push_back(r);
    }
    num_beams = sensor_ranges.size();

    if (lrf_.getNumBeams() != num_beams)
    {
        lrf_.setNumBeams(num_beams);
        lrf_.setRangeLimits(last_laser_msg_->range_min, last_laser_msg_->range_max);
        lrf_.setAngleLimits(last_laser_msg_->angle_min, last_laser_msg_->angle_max);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Create world model cross section
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<ed::EntityConstPtr> entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
//        if (it->second->id() == "pico_case")
        if (it->second->shape())
            entities.push_back(it->second);
    }

    geo::Pose3D laser_pose;
    laser_pose.t = laser_pose_.t + geo::Vector3(0, 0, 0);
    laser_pose.R.setRPY(0, 0, 0);

    std::vector<geo::Vector3> lines_start;
    std::vector<geo::Vector3> lines_end;
    LineRenderResult render_result(lines_start, lines_end);

    for(std::vector<ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        geo::LaserRangeFinder::RenderOptions options;
        geo::Transform t_inv = laser_pose.inverse() * e->pose();
        options.setMesh(e->shape()->getMesh(), t_inv);
        lrf_.render(options, render_result);
    }



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Test samples and find sample with lowest error
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Vector3> sensor_points;
    lrf_.rangesToPoints(sensor_ranges, sensor_points);

    double min_sum_sq_error = 1e10;
    for(std::vector<geo::Pose3D>::const_iterator it = poses.begin(); it != poses.end(); ++it)
    {
        const geo::Pose3D& laser_pose = *it;
        geo::Pose3D laser_pose_inv = laser_pose.inverse();

        double z1 = laser_pose_inv.R.xx;
        double z2 = laser_pose_inv.R.xy;
        double z3 = laser_pose_inv.R.yx;
        double z4 = laser_pose_inv.R.yy;
        double t1 = laser_pose_inv.t.x;
        double t2 = laser_pose_inv.t.y;

        // Calculate sensor model for this pose
        std::vector<double> model_ranges(sensor_ranges.size(), 0);

        for(unsigned int i = 0; i < lines_start.size(); ++i)
        {
            const geo::Vector3& p1 = lines_start[i];
            const geo::Vector3& p2 = lines_end[i];

            // Transform the points to the laser pose
            geo::Vector3 p1_t(z1 * p1.x + z2 * p1.y + t1, z3 * p1.x + z4 * p1.y + t2, 0);
            geo::Vector3 p2_t(z1 * p2.x + z2 * p2.y + t1, z3 * p2.x + z4 * p2.y + t2, 0);

//            std::cout << p1_t << ", " << p2_t << std::endl;

            // Render the line as if seen by the sensor
            renderLine(lrf_, p1_t, p2_t, model_ranges);
        }

        double sum_sq_error = 0;
        for(unsigned int i = 0; i < sensor_ranges.size(); ++i)
        {
//            std::cout << i << ": " << sensor_ranges[i] << " " << model_ranges[i] << std::endl;

            double diff = sensor_ranges[i] - model_ranges[i];
            if (std::abs<double>(diff) > 0.3)
                diff = 0.3;

            sum_sq_error += diff * diff;
        }

//        std::cout << sum_sq_error << std::endl;

        if (sum_sq_error < min_sum_sq_error)
        {
            min_sum_sq_error = sum_sq_error;
            best_laser_pose_ = laser_pose;
            pose_initialized_ = true;
        }
    }

    std::cout << min_sum_sq_error << std::endl;
    std::cout << best_laser_pose_ << std::endl;

    std::cout << "Num Poses = " << poses.size() << std::endl;
    std::cout << "Num lines = " << lines_start.size() << std::endl;
    std::cout << "Total time = " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
    std::cout << "Time per pose = " << timer.getElapsedTimeInMilliSec() / poses.size() << " ms" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Visualization
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    bool visualize = true;
    if (visualize)
    {
        int grid_size = 800;
        double grid_resolution = 0.025;

        cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

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


        for(unsigned int i = 0; i < lines_start.size(); ++i)
        {
            const geo::Vector3& p1 = lines_start[i];
            int mx1 = -p1.y / grid_resolution + grid_size / 2;
            int my1 = -p1.x / grid_resolution + grid_size / 2;

            const geo::Vector3& p2 = lines_end[i];
            int mx2 = -p2.y / grid_resolution + grid_size / 2;
            int my2 = -p2.x / grid_resolution + grid_size / 2;

            cv::line(rgb_image, cv::Point(mx1, my1), cv::Point(mx2, my2), cv::Scalar(255, 255, 255), 1);
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
