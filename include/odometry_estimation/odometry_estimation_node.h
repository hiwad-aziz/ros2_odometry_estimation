#ifndef ODOMETRY_ESTIMATION_NODE_H
#define ODOMETRY_ESTIMATION_NODE_H

#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"

class OdometryEstimator : public rclcpp::Node {
 public:
  OdometryEstimator();

 private:
  void handleInput();
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

#endif  // ODOMETRY_ESTIMATION_NODE_H