cmake_minimum_required(VERSION 3.5)
project(odometry_estimator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(odometry_estimator src/odometry_estimation_node.cpp
                                  src/vehicle_models.cpp)
ament_target_dependencies(odometry_estimator rclcpp nav_msgs)
target_include_directories(
  odometry_estimator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS odometry_estimator DESTINATION lib/${PROJECT_NAME})

ament_package()
