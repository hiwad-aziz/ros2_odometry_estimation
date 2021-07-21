#include "odometry_estimation/odometry_estimation_node.h"

// Constructor
OdometryEstimator::OdometryEstimator() : Node("odometry_publisher")
{
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

void OdometryEstimator::handleInput()
{
  auto message = nav_msgs::msg::Odometry();
  publisher_->publish(message);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryEstimator>());
  rclcpp::shutdown();
  return 0;
}