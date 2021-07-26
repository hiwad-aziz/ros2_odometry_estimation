#ifndef ODOMETRY_ESTIMATION_NODE_H
#define ODOMETRY_ESTIMATION_NODE_H

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64.hpp>

#include "odometry_estimation/vehicle_models.h"
#include "rclcpp/rclcpp.hpp"

class OdometryEstimator : public rclcpp::Node {
 public:
  OdometryEstimator();

 private:
  void handleRightWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_right);
  void handleLeftWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_left);
  void publish();
  VehicleModelPtr vehicle_model_;
  VehicleState state_;
  int rpm_left_;
  int rpm_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_wheel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_wheel_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ODOMETRY_ESTIMATION_NODE_H