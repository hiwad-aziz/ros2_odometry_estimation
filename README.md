
# Odometry Estimation ROS2 Package
This repository contains a ROS2 package that receives different sensor inputs (e.g. wheel rpm) and estimates odometry based on a selectable model. Currently, only a differential drive model has been implemented and forward kinematics are calculated based on input from wheel encoders.

## Dependencies
- No dependencies currently

## Setup

Build the package in your workspace:

    colcon build --packages-select odometry_estimator

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 run odometry_estimator odometry_estimator

Wheel radius and track length can be set in `vehicle_models.h`.

## Input Topics
I'm using [this](https://github.com/hiwad-aziz/ros2_f249_driver) repository with F249 sensors to get the RPM of my wheels.
- /right_wheel_rpm (std_msgs/Int64): RPM of right wheel
- /left_wheel_rpm (std_msgs/Int64): RPM of left wheel

## Output Topics
- /odom (nav_msgs/Odometry): 2D pose and orientation