ament_add_gtest(vehicle_models_test vehicle_models_test.cpp)
target_link_libraries(vehicle_models_test
    vehicle_models
)