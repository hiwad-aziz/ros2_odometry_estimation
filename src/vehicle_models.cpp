#include "vehicle_models.h"

using VehicleModelPtr = std::unique_ptr<VehicleModel>;

std::unique_ptr<VehicleModel> VehicleModel::createConcreteVehicleModel(std::string model_name)
{
  if (model_name.compare("DifferentialDrive") == 0) {
    return DifferentialDriveModel::create();
  }
}

std::unique_ptr<VehicleModel> DifferentialDriveModel::create()
{
  return VehicleModelPtr{new DifferentialDriveModel};
}