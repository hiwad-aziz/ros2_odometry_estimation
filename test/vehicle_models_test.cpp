#include <gtest/gtest.h>
#include <cmath>

#include "vehicle_models.h"

// Test cases
TEST(VehicleModels, VehicleModelFactory_ReturnDefaultModel)
{
    VehicleModelPtr vehicleModel{VehicleModel::createConcreteVehicleModel("DummyString")};
    DifferentialDriveModel* differentialDriveModel{dynamic_cast<DifferentialDriveModel*>(vehicleModel.get())};
    EXPECT_NE(differentialDriveModel, nullptr);
}

TEST(VehicleModels, VehicleModelFactory_ReturnDifferentialDriveModel)
{
    VehicleModelPtr vehicleModel{VehicleModel::createConcreteVehicleModel("DifferentialDrive")};
    DifferentialDriveModel* differentialDriveModel{dynamic_cast<DifferentialDriveModel*>(vehicleModel.get())};
    EXPECT_NE(differentialDriveModel, nullptr);
}

TEST(VehicleModels, DifferentialDriveModel_NextState)
{

}

TEST(HelperMethods, WrapAngle_LessThanZero)
{
    const double inputInRad{-1.0F};
    EXPECT_FLOAT_EQ(HelperMethods::wrapAngle(inputInRad), inputInRad + 2 * M_PI);
}

TEST(HelperMethods, WrapAngle_LessThanMinusTwoPi)
{
    const double inputInRad{-1.0F - 2 * M_PI};
    EXPECT_FLOAT_EQ(HelperMethods::wrapAngle(inputInRad), inputInRad + 4 * M_PI);
}

TEST(HelperMethods, WrapAngle_GreaterThanTwoPi)
{
    const double inputInRad{1.0F + 2 * M_PI};
    EXPECT_FLOAT_EQ(HelperMethods::wrapAngle(inputInRad), inputInRad - 2 * M_PI);    
}

TEST(HelperMethods, WrapAngle_GreaterThanFourPi)
{
    const double inputInRad{1.0F + 4 * M_PI};
    EXPECT_FLOAT_EQ(HelperMethods::wrapAngle(inputInRad), inputInRad - 4 * M_PI);    
}

TEST(HelperMethods, WrapAngle_InBounds)
{
    const double inputInRad{1.0F};
    EXPECT_FLOAT_EQ(HelperMethods::wrapAngle(inputInRad), inputInRad);      
}
