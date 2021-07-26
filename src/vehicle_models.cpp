#include "odometry_estimation/vehicle_models.h"

#include <cmath>

std::unique_ptr<VehicleModel> VehicleModel::createConcreteVehicleModel(std::string model_name)
{
  if (model_name.compare("DifferentialDrive") == 0) {
    return DifferentialDriveModel::create();
  }
  else {
    return DifferentialDriveModel::create();
  }
}

std::unique_ptr<VehicleModel> DifferentialDriveModel::create()
{
  return VehicleModelPtr{new DifferentialDriveModel};
}

DifferentialDriveModel::DifferentialDriveModel()
{
  // Intentionally blank
}

VehicleState DifferentialDriveModel::calculateNextState(int rpm_left_wheel, int rpm_right_wheel,
                                                        VehicleState prev_state, double dt)
{
  VehicleState new_state{};
  // calculate wheel velocities
  double v_l = static_cast<double>(rpm_left_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  double v_r = static_cast<double>(rpm_right_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  // calculate radius of curve
  double radius = VEHICLE_TRACK / 2 * (v_r + v_l) / (v_r - v_l);
  // calculate ICC
  double x_icc = prev_state.x - radius * std::sin(prev_state.yaw);
  double y_icc = prev_state.y + radius * std::cos(prev_state.yaw);
  // calculate angular velocity
  double angular_vel = (v_r - v_l) / VEHICLE_TRACK;
  // calculate forward kinematics
  double dyaw = angular_vel * dt;
  new_state.x =
      std::cos(dyaw) * (prev_state.x - x_icc) - std::sin(dyaw) * (prev_state.y - y_icc) + x_icc;
  new_state.y =
      std::sin(dyaw) * (prev_state.x - x_icc) + std::cos(dyaw) * (prev_state.y - y_icc) + y_icc;
  new_state.yaw = prev_state.yaw + dyaw;

  return new_state;
}