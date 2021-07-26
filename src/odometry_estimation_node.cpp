#include "odometry_estimation/odometry_estimation_node.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <functional>
#include <numeric>

using std::placeholders::_1;
using namespace std::chrono_literals;

// Constructor
OdometryEstimator::OdometryEstimator() : Node("odometry_publisher")
{
  // init vehicle model
  vehicle_model_ = VehicleModel::createConcreteVehicleModel("DifferentialDrive");

  // create subscribers
  right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      "right_wheel_rpm", 10,
      std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
  left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
      "left_wheel_rpm", 10,
      std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

  // create publisher and timer
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&OdometryEstimator::publish, this));
}

void OdometryEstimator::handleRightWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_right)
{
  rpms_right_.push_back(rpm_right->data);
}

void OdometryEstimator::handleLeftWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_left)
{
  rpms_left_.push_back(rpm_left->data);
}

void OdometryEstimator::publish()
{
  // calculate passed time since last publish
  TimePoint current_time = Clock::now();
  std::chrono::duration<double> dt = current_time - previous_time_;
  // calculate average of received rpm signals
  int rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
  int rpm_right_avg =
      std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();
  rpms_left_.clear();
  rpms_right_.clear();
  // calculate new state based on input
  VehicleState new_state =
      vehicle_model_->calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());
  // create quaternion from yaw angle
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, new_state.yaw);
  // fill message and publish
  auto message = nav_msgs::msg::Odometry();
  message.pose.pose.position.x = new_state.x;
  message.pose.pose.position.y = new_state.y;
  message.pose.pose.orientation.x = quat.x();
  message.pose.pose.orientation.y = quat.y();
  message.pose.pose.orientation.z = quat.z();
  message.pose.pose.orientation.w = quat.w();
  publisher_->publish(message);
  // update internal state
  state_ = new_state;
  previous_time_ = current_time;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryEstimator>());
  rclcpp::shutdown();
  return 0;
}