#include "localization_plugin.h"

#include "particle_filter.h"

#include <ed/entity.h>
#include <ed/plugin.h>
#include <ed/update_request.h>
#include <ed/world_model.h>

#include <geolib/Box.h>
#include <geolib/datatypes.h>
#include <geolib/math_types.h>
#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf2_conversions.h>
#include <geolib/Shape.h>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription_options.hpp>
#include <tf2/convert.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/time.hpp>
#include <tf2/transform_datatypes.hpp>
// Provides the toMsg/fromMsg overloads found by ADL below.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // IWYU pragma: keep
// tf2_ros::TransformBroadcaster must be complete for the unique_ptr member.
#include <tf2_ros/transform_broadcaster.h> // IWYU pragma: keep
#include <tue/config/configuration.h>
#include <tue/config/types.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
// drand48() is POSIX, declared by <stdlib.h>; <cstdlib> only guarantees the ISO C subset.
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <memory>
#include <numbers>
#include <stdlib.h> // NOLINT(modernize-deprecated-headers)
#include <string>
#include <system_error>
#include <utility>
#include <vector>

//! Node parameters holding an explicit robot pose in the map frame, as a launch-time override.
static const std::string INITIAL_POSE_X = "initial_pose.x";
static const std::string INITIAL_POSE_Y = "initial_pose.y";
static const std::string INITIAL_POSE_YAW = "initial_pose.yaw";

//! Default rate (Hz) at which the current pose is written to disk. 0 disables periodic saving.
static constexpr double DEFAULT_SAVE_POSE_RATE = 0.5;

namespace
{

//! $ROS_HOME when set and non-empty, else ~/.ros - the resolution ROS 2's own launch and
//! controller_manager use. Empty when neither is available, in which case nothing is persisted
//! unless the config names a file explicitly.
std::filesystem::path rosHome()
{
    const char* const ros_home = getenv("ROS_HOME");
    if (ros_home != nullptr && ros_home[0] != '\0')
        return {ros_home};

    const char* const home = getenv("HOME");
    if (home != nullptr && home[0] != '\0')
        return std::filesystem::path(home) / ".ros";

    return {};
}

//! Reduce a robot name to something safe to use as a single filename component.
std::string sanitizeForFilename(const std::string& name)
{
    std::string out = name;
    std::ranges::replace_if(out, [](unsigned char c) { return std::isalnum(c) == 0 && c != '-' && c != '_'; }, '_');
    return out;
}

//! Read x/y/rz at the config's current level. Shared by the config and the on-disk pose, which
//! deliberately use the same schema so one reader serves both.
geo::Transform2d readPoseValues(tue::Configuration& config)
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    config.value("x", x);
    config.value("y", y);
    config.value("rz", yaw);
    return {x, y, yaw};
}

} // namespace

class ConfigurationException : public std::exception
{
public:
    explicit ConfigurationException(std::string msg) : message_(std::move(msg)) {}

    [[nodiscard]]
    const char* what() const noexcept override
    {
        return message_.c_str();
    }

private:
    std::string message_;
};

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::LocalizationPlugin() :
    visualize_(false), resample_interval_(1), resample_count_(0), update_min_d_(0), update_min_a_(0),
    have_previous_odom_pose_(false), latest_map_odom_valid_(false), update_(false), laser_offset_initialized_(false),
    last_map_size_revision_(0), save_pose_interval_(0, 0), last_pose_save_(0, 0), transform_tolerance_(0, 0),
    tf_broadcaster_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

LocalizationPlugin::~LocalizationPlugin()
{
    // configure() may never have run, in which case there is nothing to save and no node to log to.
    if (!node_)
        return;

    // Nothing may escape a destructor.
    try
    {
        saveInitialPose();
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "[Localization] Could not save the pose: " << ex.what());
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::configure(tue::Configuration config)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    visualize_ = false;
    config.value("visualize", visualize_, tue::config::OPTIONAL);

    config.value("resample_interval", resample_interval_);

    config.value("update_min_d", update_min_d_);
    config.value("update_min_a", update_min_a_);

    std::string laser_topic;

    if (config.readGroup("odom_model", tue::config::REQUIRED))
    {
        config.value("map_frame", map_frame_id_);
        config.value("odom_frame", odom_frame_id_);
        config.value("base_link_frame", base_link_frame_id_);

        odom_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("laser_model", tue::config::REQUIRED))
    {
        config.value("topic", laser_topic);
        laser_model_.configure(config);
        config.endGroup();
    }

    if (config.readGroup("particle_filter", tue::config::REQUIRED))
    {
        particle_filter_.configure(config);
        config.endGroup();
    }

    double tmp_transform_tolerance = 0.1;
    config.value("transform_tolerance", tmp_transform_tolerance, tue::config::OPTIONAL);
    transform_tolerance_ = rclcpp::Duration::from_seconds(tmp_transform_tolerance);

    // Read before resolveInitialPoseFile(), which keys the default filename by robot name.
    config.value("robot_name", robot_name_);

    double save_pose_rate = DEFAULT_SAVE_POSE_RATE;
    config.value("save_pose_rate", save_pose_rate, tue::config::OPTIONAL);
    save_pose_interval_ =
        save_pose_rate > 0.0 ? rclcpp::Duration::from_seconds(1.0 / save_pose_rate) : rclcpp::Duration(0, 0);

    resolveInitialPoseFile(config);

    if (config.hasError())
        return;

    last_pose_save_ = node_->now();

    // A plugin does not own the node's executor, so it pumps its own callbacks from process().
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    // Subscribe to laser topic
    sub_laser_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic,
        1,
        [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) { laserCallback(msg); },
        sub_options);

    std::string initial_pose_topic;
    if (config.value("initial_pose_topic", initial_pose_topic, tue::config::OPTIONAL))
    {
        // Subscribe to initial pose topic
        sub_initial_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic,
            1,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
            { initialPoseCallback(msg); },
            sub_options);
    }

    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    geo::Transform2d const initial_pose = getInitialPose(config);

    initParticleFilterUniform(initial_pose);

    pub_particles_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("ed/localization/particles", 10);
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2d LocalizationPlugin::getInitialPose(tue::Configuration& config)
{
    // Every source is optional and logs why it could not supply a pose; fall through to the next,
    // and to identity if none has anything. An explicit parameter (a launch-time override) beats the
    // pose recovered from the previous run, which beats the static default in the config.
    try
    {
        return tryGetInitialPoseFromParamServer();
    }
    catch (const ConfigurationException& ex)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[Localization] " << ex.what());
    }

    try
    {
        return tryGetInitialPoseFromFile();
    }
    catch (const ConfigurationException& ex)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[Localization] " << ex.what());
    }

    try
    {
        return tryGetInitialPoseFromConfig(config);
    }
    catch (const ConfigurationException& ex)
    {
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "[Localization] " << ex.what());
    }

    return geo::Transform2d::identity();
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::tryGetInitialPoseFromParamServer()
{
    // In ROS 1 these parameters lived on the global parameter server and held the map -> odom
    // offset, which this plugin wrote on shutdown and composed with a live odom -> base_link lookup
    // on start-up. ROS 2 has no global parameter server, so persistence moved to a file (see
    // tryGetInitialPoseFromFile) and these parameters are now purely a launch-time override holding
    // the robot pose in the map frame - the same meaning as the config group and the stored file.
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!node_->get_parameter(INITIAL_POSE_X, x) || !node_->get_parameter(INITIAL_POSE_Y, y) ||
        !node_->get_parameter(INITIAL_POSE_YAW, yaw))
        throw ConfigurationException("No initial pose set on the parameters");

    geo::Transform2d const result(x, y, yaw);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "[Localization] Initial pose from parameters: [" << result.t.x << ", " << result.t.y
                                                                         << "], yaw:" << result.rotation());
    return result;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::resolveInitialPoseFile(tue::Configuration& config)
{
    std::string configured;
    if (config.value("initial_pose_file", configured, tue::config::OPTIONAL) && !configured.empty())
    {
        initial_pose_file_ = configured;
        return;
    }

    std::filesystem::path const ros_home = rosHome();
    if (ros_home.empty())
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[Localization] Neither ROS_HOME nor HOME is set and no 'initial_pose_file' is configured; "
                    "the pose will not survive a restart");
        return;
    }

    // Keyed by robot name: ED runs one instance per robot, and two robots sharing a home directory
    // would otherwise overwrite each other's pose.
    std::string const name = robot_name_.empty() ? "initial_pose" : sanitizeForFilename(robot_name_);
    initial_pose_file_ = ros_home / "ed_localization" / (name + ".yaml");
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[Localization] Pose file: " << initial_pose_file_.string());
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::tryGetInitialPoseFromFile()
{
    if (initial_pose_file_.empty())
        throw ConfigurationException("No initial pose file to read");

    std::error_code ec;
    if (!std::filesystem::exists(initial_pose_file_, ec))
        throw ConfigurationException("No stored pose at '" + initial_pose_file_.string() + "'");

    tue::Configuration stored;
    if (!stored.loadFromYAMLFile(initial_pose_file_.string()))
        throw ConfigurationException("Could not read '" + initial_pose_file_.string() + "': " + stored.error());

    if (!stored.readGroup("initial_pose", tue::config::REQUIRED))
        throw ConfigurationException("No 'initial_pose' group in '" + initial_pose_file_.string() + "'");

    std::string stored_map_frame;
    stored.value("map_frame", stored_map_frame, tue::config::OPTIONAL);
    geo::Transform2d const result = readPoseValues(stored);
    stored.endGroup();

    if (stored.hasError())
        throw ConfigurationException("Invalid stored pose in '" + initial_pose_file_.string() + "': " + stored.error());

    // A pose only means something against the map it was recorded in. initParticleFilterUniform()
    // seeds a narrow window around it, so a pose from another environment is a confidently wrong
    // start - worse than falling back to the config default.
    if (stored_map_frame != map_frame_id_)
        throw ConfigurationException("Stored pose is in frame '" + stored_map_frame + "' but this run uses '" +
                                     map_frame_id_ + "'");

    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "[Localization] Initial pose from '" << initial_pose_file_.string() << "': [" << result.t.x
                                                             << ", " << result.t.y << "], yaw:" << result.rotation());
    return result;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::tryGetInitialPoseFromConfig(tue::Configuration& config)
{
    if (!config.readGroup("initial_pose", tue::config::OPTIONAL))
        throw ConfigurationException("Initial pose not present in config");

    geo::Transform2d const result = readPoseValues(config);
    config.endGroup();

    return result;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::saveInitialPose()
{
    if (initial_pose_file_.empty() || !latest_map_odom_valid_ || !have_previous_odom_pose_)
        return;

    // Stored as the robot pose in the map frame, the same meaning as the 'initial_pose' config group.
    // Note this is a snapshot: if the robot is moved while ED is down, the recovered pose is stale.
    // Storing map -> odom instead would survive that, but only while the odometry source keeps
    // running - across a reboot the odom frame resets and such an offset is meaningless.
    geo::Transform2 const pose = (latest_map_odom_ * previous_odom_pose_).projectTo2d();

    tue::Configuration out;
    out.writeGroup("initial_pose");
    out.setValue("x", pose.t.x);
    out.setValue("y", pose.t.y);
    out.setValue("rz", pose.rotation());
    out.setValue("map_frame", map_frame_id_);
    out.endGroup();

    std::error_code ec;
    std::filesystem::create_directories(initial_pose_file_.parent_path(), ec);
    if (ec)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "[Localization] Could not create '" << initial_pose_file_.parent_path().string()
                                                                << "': " << ec.message());
        return;
    }

    // Write to a sibling file and rename: a crash or power cut mid-write then leaves the previous
    // pose intact instead of a truncated file.
    std::filesystem::path const tmp_file = initial_pose_file_.string() + ".tmp";
    {
        std::ofstream out_stream(tmp_file, std::ios::trunc);
        out_stream << out.toYAMLString();
        if (!out_stream)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "[Localization] Could not write '" << tmp_file.string() << "'");
            return;
        }
    }

    std::filesystem::rename(tmp_file, initial_pose_file_, ec);
    if (ec)
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "[Localization] Could not move '" << tmp_file.string() << "' into place: " << ec.message());
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialize() {}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    initial_pose_msg_.reset();
    executor_.spin_some();

    if (initial_pose_msg_)
    {
        // Set initial pose
        geo::Pose3D pose;
        geo::convert(initial_pose_msg_->pose.pose, pose);
        initParticleFilterUniform(pose.projectTo2d());
    }

    while (!scan_buffer_.empty())
    {
        TransformStatus const status = update(scan_buffer_.front(), world, req);
        if (status == OK || status == TOO_OLD || status == UNKNOWN_ERROR)
            scan_buffer_.pop();
        else
            break;
    }

    // Save periodically rather than only from the destructor, which does not run on a crash, a
    // SIGKILL or a power cut - exactly the cases the stored pose is meant to survive.
    if (save_pose_interval_ > rclcpp::Duration(0, 0) && (node_->now() - last_pose_save_) >= save_pose_interval_)
    {
        saveInitialPose();
        last_pose_save_ = node_->now();
    }
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::update(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                                           const ed::WorldModel& world,
                                           ed::UpdateRequest& req)
{
    //  Get transformation from base_link to laser_frame
    if (!laser_offset_initialized_)
    {
        TransformStatus const ts = initLaserOffset(scan->header.frame_id, scan->header.stamp);
        if (ts != OK)
            return ts;
    }

    // Check if particle filter is initialized
    if (particle_filter_.samples().empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "[Localization] (update) Empty particle filter");
        return UNKNOWN_ERROR;
    }

    // Calculate delta movement based on odom (fetched from TF)
    geo::Pose3D odom_to_base_link;
    geo::Transform2 movement;

    tf2::Stamped<tf2::Transform> odom_to_base_link_tf;
    TransformStatus const ts = transform(odom_frame_id_, base_link_frame_id_, scan->header.stamp, odom_to_base_link_tf);
    if (ts != OK)
        return ts;

    geo::convert(odom_to_base_link_tf, odom_to_base_link);

    if (have_previous_odom_pose_)
    {
        // Get displacement and project to 2D
        movement = (previous_odom_pose_.inverse() * odom_to_base_link).projectTo2d();

        update_ = std::abs(movement.t.x) >= update_min_d_ || std::abs(movement.t.y) >= update_min_d_ ||
                  std::abs(movement.rotation()) >= update_min_a_;
    }

    bool force_publication = false;
    if (!have_previous_odom_pose_)
    {
        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;
        update_ = true;
        force_publication = true;
    }
    else if (have_previous_odom_pose_ && update_)
    {
        // Update motion
        odom_model_.updatePoses(movement, particle_filter_);
    }

    bool resampled = false;
    if (update_)
    {
        RCLCPP_DEBUG(node_->get_logger(), "[Localization] Updating laser");
        // Update sensor
        laser_model_.updateWeights(world, *scan, particle_filter_);

        previous_odom_pose_ = odom_to_base_link;
        have_previous_odom_pose_ = true;

        update_ = false;

        // (Re)sample
        resampled = resample(world);

        // Publish particles
        publishParticles(scan->header.stamp);
    }

    // Update map-odom
    if (resampled || force_publication)
    {
        updateMapOdom(odom_to_base_link);
    }

    // Publish result
    if (latest_map_odom_valid_)
    {
        publishMapOdom(scan->header.stamp);

        // This should be executed allways. map_odom * odom_base_link
        if (!robot_name_.empty())
            req.setPose(robot_name_, latest_map_odom_ * previous_odom_pose_);
    }

    // Visualization
    if (visualize_)
    {
        visualize();
    }
    else
    {
        cv::destroyAllWindows();
    }

    return OK;
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::initLaserOffset(const std::string& frame_id, const rclcpp::Time& stamp)
{
    tf2::Stamped<tf2::Transform> p_laser;
    TransformStatus const ts = transform(base_link_frame_id_, frame_id, stamp, p_laser);

    if (ts != OK)
        return ts;

    geo::Transform2 offset(
        geo::Mat2(
            p_laser.getBasis()[0][0], p_laser.getBasis()[0][1], p_laser.getBasis()[1][0], p_laser.getBasis()[1][1]),
        geo::Vec2(p_laser.getOrigin().getX(), p_laser.getOrigin().getY()));

    bool const upside_down = p_laser.getBasis()[2][2] < 0;
    if (upside_down)
    {
        offset.R.yx = -offset.R.yx;
        offset.R.yy = -offset.R.yy;
    }

    double const laser_height = p_laser.getOrigin().getZ();

    laser_model_.setLaserOffset(offset, laser_height, upside_down);

    laser_offset_initialized_ = true;

    return OK;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initParticleFilterUniform(const geo::Transform2& pose)
{
    const geo::Vec2& p = pose.getOrigin();
    const double yaw = pose.rotation();
    particle_filter_.initUniform(p - geo::Vec2(0.3, 0.3), p + geo::Vec2(0.3, 0.3), yaw - 0.3, yaw + 0.3);
    have_previous_odom_pose_ = false;
    resample_count_ = 0;
}

// ----------------------------------------------------------------------------------------------------

bool LocalizationPlugin::resample(const ed::WorldModel& world)
{
    if (++resample_count_ % resample_interval_)
        return false;

    RCLCPP_DEBUG(node_->get_logger(), "[Localization] resample particle filter");
    const std::function<void()> update_map_size_func = [this, &world]() { updateMapSize(world); };
    const std::function<geo::Transform2()> gen_random_pose_func = [this, &update_map_size_func]()
    { return generateRandomPose(update_map_size_func); };
    particle_filter_.resample(gen_random_pose_func);
    return true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::publishParticles(const rclcpp::Time& stamp)
{
    RCLCPP_DEBUG(node_->get_logger(), "[Localization] Publishing particles");
    const std::vector<Sample>& samples = particle_filter_.samples();
    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.poses.resize(samples.size());
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
        const geo::Transform2& p = samples[i].pose;

        geo::Pose3D const pose_3d = p.projectTo3d();

        geo::convert(pose_3d, particles_msg.poses[i]);
    }

    particles_msg.header.frame_id = map_frame_id_;
    particles_msg.header.stamp = stamp;

    pub_particles_->publish(particles_msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::updateMapOdom(const geo::Pose3D& odom_to_base_link)
{
    RCLCPP_DEBUG(node_->get_logger(), "[Localization] Updating map_odom");
    // Get the best pose (2D)
    geo::Transform2 const mean_pose = particle_filter_.calculateMeanPose();
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
                        "[Localization] mean_pose: x: " << mean_pose.t.x << ", y: " << mean_pose.t.y
                                                        << ", yaw: " << mean_pose.rotation());

    // Convert best pose to 3D
    geo::Pose3D map_to_base_link;
    map_to_base_link = mean_pose.projectTo3d();

    latest_map_odom_ = map_to_base_link * odom_to_base_link.inverse();
    latest_map_odom_valid_ = true;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::publishMapOdom(const rclcpp::Time& stamp)
{
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[Localization] Publishing map_odom");
    // Convert to TF transform
    geometry_msgs::msg::TransformStamped latest_map_odom_tf;
    geo::convert(latest_map_odom_, latest_map_odom_tf.transform);

    // Set frame id's and time stamp
    latest_map_odom_tf.header.frame_id = map_frame_id_;
    latest_map_odom_tf.child_frame_id = odom_frame_id_;
    latest_map_odom_tf.header.stamp = stamp + transform_tolerance_;

    // Publish TF
    tf_broadcaster_->sendTransform(latest_map_odom_tf);
}

// ----------------------------------------------------------------------------------------------------

TransformStatus LocalizationPlugin::transform(const std::string& target_frame,
                                              const std::string& source_frame,
                                              const rclcpp::Time& time,
                                              tf2::Stamped<tf2::Transform>& transform)
{
    try
    {
        geometry_msgs::msg::TransformStamped const ts = tf_buffer_->lookupTransform(target_frame, source_frame, time);
        tf2::convert(ts, transform);
        return OK;
    }
    catch (const tf2::ExtrapolationException&)
    {
        try
        {
            // Now we have to check if the error was an interpolation or extrapolation error
            // (i.e., the scan is too old or too new, respectively)
            geometry_msgs::msg::TransformStamped const latest_transform =
                tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            if (rclcpp::Time(scan_buffer_.front()->header.stamp) > rclcpp::Time(latest_transform.header.stamp))
            {
                // Scan is too new
                return TOO_RECENT;
            }

            // Otherwise it has to be too old
            return TOO_OLD;
        }
        catch (const tf2::TransformException&)
        {
            return UNKNOWN_ERROR;
        }
    }
    catch (const tf2::TransformException&)
    {
        return UNKNOWN_ERROR;
    }
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
    scan_buffer_.push(msg);
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
    initial_pose_msg_ = msg;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 LocalizationPlugin::generateRandomPose(const std::function<void()>& update_map_size)
{
    update_map_size();
    geo::Transform2 pose;
    pose.t = min_map_;
    const geo::Vec2 map_size = max_map_ - min_map_;
    pose.t.x += drand48() * map_size.x;
    pose.t.y += drand48() * map_size.y;
    pose.setRotation((drand48() * 2 * std::numbers::pi) - std::numbers::pi);
    return pose;
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::updateMapSize(const ed::WorldModel& world)
{
    if (world.revision() <= last_map_size_revision_)
        return;

    geo::Vec2 min(1e6, 1e6);
    geo::Vec2 max(-1e6, -1e6);

    for (const auto& e : world)
    {
        const geo::ShapeConstPtr& shape = e->visual();

        // Skip robot and entities without pose or shape
        if (!e->hasPose() || !shape || e->hasFlag("self") || shape->getBoundingBox().getMax().z < 0.05)
            continue;

        geo::Vector3 const min_entity_world = e->pose() * shape->getBoundingBox().getMin();
        geo::Vector3 const max_entity_world = e->pose() * shape->getBoundingBox().getMax();

        min.x = std::min<double>(min.x, min_entity_world.x);
        min.y = std::min<double>(min.y, min_entity_world.y);
        max.x = std::max<double>(max.x, max_entity_world.x);
        max.y = std::max<double>(max.y, max_entity_world.y);
    }

    min_map_ = min;
    max_map_ = max;
    last_map_size_revision_ = world.revision();
}

// ----------------------------------------------------------------------------------------------------

void LocalizationPlugin::visualize()
{
    RCLCPP_DEBUG(node_->get_logger(), "[Localization] Visualize");
    int const grid_size = 800;
    double const grid_resolution = 0.025;

    cv::Mat rgb_image(grid_size, grid_size, CV_8UC3, cv::Scalar(10, 10, 10));

    std::vector<geo::Vector3> sensor_points;
    laser_model_.renderer().rangesToPoints(laser_model_.sensor_ranges(), sensor_points);

    geo::Transform2 const best_pose = (latest_map_odom_ * previous_odom_pose_).projectTo2d();

    geo::Transform2 const laser_pose = best_pose * laser_model_.laser_offset();
    for (const auto& sensor_point : sensor_points)
    {
        const geo::Vec2& p = laser_pose * geo::Vec2(sensor_point.x, sensor_point.y);
        int const mx = static_cast<int>((-(p.y - best_pose.t.y) / grid_resolution) + (grid_size / 2.0));
        int const my = static_cast<int>((-(p.x - best_pose.t.x) / grid_resolution) + (grid_size / 2.0));

        if (mx >= 0 && my >= 0 && mx < grid_size && my < grid_size)
        {
            rgb_image.at<cv::Vec3b>(my, mx) = cv::Vec3b(0, 255, 0);
        }
    }

    const std::vector<geo::Vec2>& lines_start = laser_model_.lines_start();
    const std::vector<geo::Vec2>& lines_end = laser_model_.lines_end();

    for (unsigned int i = 0; i < lines_start.size(); ++i)
    {
        const geo::Vec2& p1 = lines_start[i];
        int const mx1 = static_cast<int>((-(p1.y - best_pose.t.y) / grid_resolution) + (grid_size / 2.0));
        int const my1 = static_cast<int>((-(p1.x - best_pose.t.x) / grid_resolution) + (grid_size / 2.0));

        const geo::Vec2& p2 = lines_end[i];
        int const mx2 = static_cast<int>((-(p2.y - best_pose.t.y) / grid_resolution) + (grid_size / 2.0));
        int const my2 = static_cast<int>((-(p2.x - best_pose.t.x) / grid_resolution) + (grid_size / 2.0));

        cv::line(rgb_image, cv::Point(mx1, my1), cv::Point(mx2, my2), cv::Scalar(255, 255, 255), 1);
    }

    const std::vector<Sample>& samples = particle_filter_.samples();
    for (const auto& sample : samples)
    {
        const geo::Transform2& pose = sample.pose;

        // Visualize sensor
        int const lmx = static_cast<int>((-(pose.t.y - best_pose.t.y) / grid_resolution) + (grid_size / 2.0));
        int const lmy = static_cast<int>((-(pose.t.x - best_pose.t.x) / grid_resolution) + (grid_size / 2.0));
        cv::circle(rgb_image, cv::Point(lmx, lmy), static_cast<int>(0.1 / grid_resolution), cv::Scalar(0, 0, 255), 1);

        geo::Vec2 const d = pose.R * geo::Vec2(0.2, 0);
        int const dmx = static_cast<int>(-d.y / grid_resolution);
        int const dmy = static_cast<int>(-d.x / grid_resolution);
        cv::line(rgb_image, cv::Point(lmx, lmy), cv::Point(lmx + dmx, lmy + dmy), cv::Scalar(0, 0, 255), 1);
    }

    cv::imshow("localization", rgb_image);
    cv::waitKey(1);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(LocalizationPlugin)
