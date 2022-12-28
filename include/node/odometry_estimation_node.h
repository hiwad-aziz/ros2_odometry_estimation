#ifndef ODOMETRY_ESTIMATION_NODE_H
#define ODOMETRY_ESTIMATION_NODE_H

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64.hpp>
#include <vector>

#include "vehicle_models.h"
#include "rclcpp/rclcpp.hpp"

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

class OdometryEstimator : public rclcpp::Node {
 public:
  OdometryEstimator();

 private:
  void handleRightWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_right);
  void handleLeftWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_left);
  void publish();
  VehicleModelPtr vehicle_model_{nullptr};
  VehicleState state_{0.0, 0.0, 0.0};
  std::vector<int> rpms_left_;
  std::vector<int> rpms_right_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_wheel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_wheel_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  TimePoint previous_time_{};
};

#endif  // ODOMETRY_ESTIMATION_NODE_H