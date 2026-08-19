# ED Localization

[![CI](https://github.com/tue-robotics/ed_localization/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/ed_localization/actions/workflows/main.yml)

A fast particle filter implementation and sensor models for localizing a robot which always take into account the most recent state of the world. This means that if the world representation improves while the robot is running, localization becomes better. The localization module is more efficient and accurate than the well-known [AMCL-module](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl) and *no* separate occupancy grid is needed.

## Installation

Requirements:

* Ubuntu (24.04 or newer)
* ROS 2 (Jazzy or newer)
* ED (<https://github.com/tue-robotics/ed.git>)
* A 2D Range Finder which scans in a plane parallel to the floor
* A [TF](https://docs.ros.org/en/rolling/p/tf2/) tree containing transforms from the robots' odometry frame to the
  laser range finder frame

We assume you have successfully installed ROS 2 and set-up a colcon workspace. Check out the following packages in
your workspace:

    cd <your_colcon_workspace>/src
    git clone https://github.com/tue-robotics/ed_localization.git

Install the dependencies by resolving them via `rosdep`, then compile:

    cd <your_colcon_workspace>
    colcon build --packages-up-to ed_localization

Running the tests additionally needs the tue-robotics [`ament_lint`](https://github.com/tue-robotics/ament_lint) fork
and [`tue_lint_config`](https://github.com/tue-robotics/tue_lint_config) in the workspace, since the linters are
invoked with options that are not in upstream `ament_cmake_clang_format`/`ament_cmake_clang_tidy`:

    colcon test --packages-select ed_localization && colcon test-result --verbose

## Plugins

Both plugins are ED plugins, loaded by the ED server through `pluginlib` and configured from ED's own configuration
file:

* `ed_localization/localization` — particle filter localization against the world model, publishing the
  `map` -> `odom` transform and the particle cloud on `ed/localization/particles`.
* `ed_localization/localization_tf` — sets the robot entity pose from an externally provided TF tree.

### Remembering the pose across restarts

`ed_localization/localization` periodically writes the robot pose in the map frame to disk and reads
it back on start-up, so a restarted ED resumes where it left off instead of at the origin. In ROS 1
this used the global parameter server, which ROS 2 does not have.

| Parameter | Default | Meaning |
|---|---|---|
| `initial_pose_file` | `${ROS_HOME:-~/.ros}/ed_localization/<robot_name>.yaml` | Where the pose is stored |
| `save_pose_rate` | `0.5` | Save rate in Hz. `0` saves only on clean shutdown |

The default follows ROS 2's own `ROS_HOME`/`~/.ros` convention and is keyed by `robot_name`, so two
robots sharing a home directory do not overwrite each other. Set `initial_pose_file` explicitly when
ED runs as a system user with no usable `$HOME` — for example a systemd unit with
`StateDirectory=ed_localization`, giving `/var/lib/ed_localization`.

The file uses the same schema as the `initial_pose` config group, plus the frame it was recorded in:

```yaml
initial_pose:
    map_frame: map
    rz: -0.0542
    x: -0.0503
    y: -0.0578
```

A stored pose is rejected when `map_frame` does not match the configured `map_frame`, because seeding
the filter with a pose from a different environment is a narrow, confidently wrong start — worse than
falling back to the config default. Writes go to a sibling `.tmp` file and are renamed into place, so
a crash mid-write leaves the previous pose intact rather than a truncated file.

The initial pose is taken from the first source that has one: the `initial_pose.x`/`.y`/`.yaw` node
parameters (a launch-time override), then the stored file, then the `initial_pose` group in the ED
config, and finally the origin. All four express the robot pose in the map frame.

## Tutorial

All ED tutorials can be found in the ed_tutorials package: <https://github.com/tue-robotics/ed_tutorials.git>
