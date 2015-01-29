#include "relative_localization_plugin.h"

#include <ros/node_handle.h>
#include <ros/subscribe_options.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

class LineRenderResult : public geo::LaserRangeFinder::RenderResult
{

public:

    LineRenderResult(std::vector<geo::Vec2>& lines_start, std::vector<geo::Vec2>& lines_end)
        : geo::LaserRangeFinder::RenderResult(dummy_ranges_),
          lines_start_(lines_start), lines_end_(lines_end) {}

    void renderLine(const geo::Vec2& p1, const geo::Vec2& p2)
    {
        lines_start_.push_back(p1);
        lines_end_.push_back(p2);
    }

private:

    std::vector<double> dummy_ranges_;
    std::vector<geo::Vec2>& lines_start_;
    std::vector<geo::Vec2>& lines_end_;

};

// ----------------------------------------------------------------------------------------------------

RelativeLocalizationPlugin::RelativeLocalizationPlugin() : have_previous_pose_(false), tf_listener_(), tf_broadcaster_(0)
{
}

// ----------------------------------------------------------------------------------------------------

RelativeLocalizationPlugin::~RelativeLocalizationPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void RelativeLocalizationPlugin::updateLRFRenderer(const sensor_msgs::LaserScan& scan, std::vector<double>& sensor_ranges)
{
    if (num_beams_ <= 0)
        num_beams_ = scan.ranges.size();
    else
        num_beams_ = std::min<int>(scan.ranges.size(), num_beams_);

    int i_step = scan.ranges.size() / num_beams_;
    sensor_ranges.clear();
    for (unsigned int i = 0; i < scan.ranges.size(); i += i_step)
    {
        double r = scan.ranges[i];

        // Check for Inf
        if (r != r || r > scan.range_max)
            r = 0;
        sensor_ranges.push_back(r);
    }
    num_beams_ = sensor_ranges.size();

    if (lrf_.getNumBeams() != num_beams_)
    {
        lrf_.setNumBeams(num_beams_);
        lrf_.setAngleLimits(scan.angle_min, scan.angle_max);
        range_max_ = std::min<double>(range_max_, scan.range_max);

        // Convert 3D ray directions from lrf object to 2D ray directions
        const std::vector<geo::Vector3>& ray_directions = lrf_.rayDirections();
        ray_directions_.resize(ray_directions.size());
        for(unsigned int i = 0; i < ray_directions.size(); ++i)
        {
            const geo::Vector3& v = ray_directions[i];
            ray_directions_[i] = geo::vec2(v.x, v.y);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void RelativeLocalizationPlugin::configure(tue::Configuration config)
{
    std::string laser_topic;

    num_beams_ = 100;
    range_max_ = 10;

    if (config.readGroup("odom_model", tue::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        config.endGroup();
    }

    if (config.readGroup("laser_model", tue::REQUIRED))
    {
        config.value("topic", laser_topic);

        config.endGroup();
    }

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    // Subscribe to laser topic
    ros::SubscribeOptions sub_options =
            ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
                laser_topic, 1, boost::bind(&RelativeLocalizationPlugin::laserCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    sub_laser_ = nh.subscribe(sub_options);

    delete tf_listener_;
    tf_listener_ = new tf::TransformListener;

    delete tf_broadcaster_;
    tf_broadcaster_ = new tf::TransformBroadcaster;
}

// ----------------------------------------------------------------------------------------------------

void RelativeLocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    laser_msg_.reset();
    cb_queue_.callAvailable();

    if (!laser_msg_)
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Calculate delta movement based on odom (fetched from TF)
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (!tf_listener_->waitForTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, ros::Duration(1.0)))
    {
        ROS_WARN_STREAM("[ED LOCALIZATION] Cannot get transform from '" << odom_frame_id_ << "' to '" << base_link_frame_id_ << "'.");
        return;
    }

    geo::Pose3D odom_to_base_link;
    geo::Transform2 delta_2d;

    try
    {
        tf::StampedTransform odom_to_base_link_tf;

        tf_listener_->lookupTransform(odom_frame_id_, base_link_frame_id_, laser_msg_->header.stamp, odom_to_base_link_tf);

        geo::convert(odom_to_base_link_tf, odom_to_base_link);

        if (have_previous_pose_)
        {
            geo::Pose3D delta = previous_pose_.inverse() * odom_to_base_link;

            // Convert to 2D transformation
            delta_2d = geo::Transform2(geo::Mat2(delta.R.xx, delta.R.xy,
                                                 delta.R.yx, delta.R.yy),
                                       geo::Vec2(delta.t.x, delta.t.y));

        }
        else
        {
            delta_2d = geo::Transform2::identity();
        }

        previous_pose_ = odom_to_base_link;
        have_previous_pose_ = true;
    }
    catch (tf::TransformException e)
    {
        std::cout << "[ED LOCALIZATION] " << e.what() << std::endl;

        if (!have_previous_pose_)
            return;

        odom_to_base_link = previous_pose_;
        delta_2d = geo::Transform2::identity();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Update world renderer
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<double> sensor_ranges;
    updateLRFRenderer(*laser_msg_, sensor_ranges);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // -     Localize entity
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ed::UUID entity_id = "bla";
    ed::EntityConstPtr e = world.getEntity(entity_id);
    if (!e)
    {
        std::cout << "[ED LOCALIZATION] No such entity: '" << entity_id << "'." << std::endl;
        return;
    }

    if (!e->shape())
    {
        std::cout << "[ED LOCALIZATION] Entity: '" << entity_id << "' has no shape." << std::endl;
        return;
    }

    // Render entity (obtain lines)
    std::vector<geo::Vec2> lines_start, lines_end;
    LineRenderResult render_result(lines_start, lines_end);

    geo::LaserRangeFinder::RenderOptions options;
    options.setMesh(e->shape()->getMesh(), geo::Pose3D(0, 0, 0.3 - e->pose().t.z)); // TODO: remove hard-coded
    lrf_.render(options, render_result);

    for(unsigned int i = 0; i < ray_directions_.size(); i += 10)
    {
        // Guess the distance to the entity, if it were positioned at this angle
        if (sensor_ranges[i] == 0 || sensor_ranges[i] > range_max_)
            continue;

        double init_distance = sensor_ranges[i] + e->shape()->getMesh().getMaxRadius();

        geo::Vec2 pos = ray_directions_[i] * init_distance;

        for(double a = 0;)

        // Calculate sensor model for this pose
        std::vector<double> model_ranges(sensor_ranges.size(), 0);

        for(unsigned int i = 0; i < lines_start.size(); ++i)
        {
            const geo::Vec2& p1 = lines_start[i];
            const geo::Vec2& p2 = lines_end[i];

            // Transform the points to the laser pose
            geo::Vec2 p1_t = pose_inv * p1;
            geo::Vec2 p2_t = pose_inv * p2;

            // Render the line as if seen by the sensor
            lrf_.renderLine(p1_t, p2_t, model_ranges);
        }
    }



//    tue::Timer timer;
//    timer.start();



//    std::cout << "----------" << std::endl;
//    std::cout << "Number of lines = " << laser_model_.lines_start().size() << std::endl;
//    std::cout << "Total time = " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
//    std::cout << "Time per sample = " << timer.getElapsedTimeInMilliSec() / particle_filter_.samples().size() << " ms" << std::endl;

}

// ----------------------------------------------------------------------------------------------------

void RelativeLocalizationPlugin::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    laser_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RelativeLocalizationPlugin)
