cmake_minimum_required(VERSION 3.16)
project(sensor_plugins)

# Buscar dependencias ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Buscar dependencias de Gazebo Fortress
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Plugin
add_library(imu_plugin SHARED src/imu_plugin.cpp)
target_include_directories(imu_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Enlazar dependencias
target_link_libraries(imu_plugin
  gz-sim7::gz-sim7
  gz-plugin2::gz-plugin2
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(imu_plugin
  rclcpp
  sensor_msgs
)

install(TARGETS imu_plugin DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()