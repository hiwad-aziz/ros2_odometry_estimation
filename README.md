
# Odometry Estimation ROS2 Package
This repository contains a ROS2 package that receives different sensor inputs (e.g. wheel rpm) and estimates odometry based on a selectable model. Currently, only a differential drive model has been implemented.

## Dependencies
- No dependencies currently

## Setup

Build the package in your workspace:

    colcon build --packages-select odometry_estimator

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 run odometry_estimator odometry_estimator

