#ifndef VEHICLE_MODELS_H
#define VEHICLE_MODELS_H

#include <memory>

// Pure virtual interface for vehicle models
class VehicleModel {
 public:
  static std::unique_ptr<VehicleModel> createConcreteVehicleModel(std::string model_name);
  virtual void calculateNextState() = 0;
}

class DifferentialDriveModel : public VehicleModel {
 public:
  void calculateNextState() override;

 private:
  DifferentialDriveModel();
  static std::unique_ptr<VehicleModel> create();

  friend VehicleModel;
}

#endif  // VEHICLE_MODELS_H