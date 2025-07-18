cmake_minimum_required(VERSION 3.8)
project(force_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_utils REQUIRED)

include_directories(
  include
)

add_library(force_plugin SHARED
  src/force_plugin.cpp
)

ament_target_dependencies(force_plugin
  rclcpp
  gazebo_ros
  sensor_utils
)

target_link_libraries(force_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS force_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()