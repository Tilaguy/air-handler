cmake_minimum_required(VERSION 3.8)
project(imu_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_utils REQUIRED)

include_directories(
  include
)

add_library(imu_plugin SHARED
  src/imu_plugin.cpp
)

ament_target_dependencies(imu_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  sensor_utils
)

target_link_libraries(imu_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS imu_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
