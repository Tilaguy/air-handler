cmake_minimum_required(VERSION 3.10)
project(imu_plugin)

# Compiler config
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/imu_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  gazebo_ros
)

# Gazebo plugin install
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Export plugin
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
