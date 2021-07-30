#ifndef VEHICLE_MODELS_H
#define VEHICLE_MODELS_H

#include <memory>

class VehicleModel;
using VehicleModelPtr = std::unique_ptr<VehicleModel>;

constexpr double WHEEL_RADIUS = 0.0335;
constexpr double VEHICLE_TRACK = 0.134;

struct VehicleState {
  double x;
  double y;
  double yaw;
};

// Pure virtual interface for vehicle models
class VehicleModel {
 public:
  static std::unique_ptr<VehicleModel> createConcreteVehicleModel(std::string model_name);
  virtual VehicleState calculateNextState(int rpm_left_wheel, int rpm_right_wheel,
                                          VehicleState prev_state, double dt) = 0;
};

class DifferentialDriveModel : public VehicleModel {
 public:
  VehicleState calculateNextState(int rpm_left_wheel, int rpm_right_wheel, VehicleState prev_state,
                                  double dt) override;

 private:
  DifferentialDriveModel();
  static std::unique_ptr<VehicleModel> create();

  friend VehicleModel;
};

class HelperMethods {
 public:
  static double wrapAngle(double angle);
};

#endif  // VEHICLE_MODELS_H