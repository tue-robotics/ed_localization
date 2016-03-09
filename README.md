ED Localization [![Build Status](https://travis-ci.org/tue-robotics/ed_localization.svg?branch=master)](https://travis-ci.org/tue-robotics/ed_localization)
======

A fast particle filter implementation and sensor models for localizing a robot which always take into account the most recent state of the world. This means that if the world representation improves while the robot is running, localization becomes better. The localization module is more efficient and accurate than the well-known [AMCL-module](http://wiki.ros.org/amcl) and *no* separate occupancy grid is needed.

## Installation

Requirements:
* ED (https://github.com/tue-robotics/ed.git)
* A 2D Range Finder (http://wiki.ros.org/Sensors) which scans in a plane parallel to the floor
* A [TF](wiki.ros.org/tf) containing transforms from the robots' odometry frame to the laser range finder frame

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_localization.git

And compile

    cd <your_catkin_workspace>:
    catkin_make
    
## Tutorial

All ED tutorials can be found in the ed_tutorials package: https://github.com/tue-robotics/ed_tutorials.git
