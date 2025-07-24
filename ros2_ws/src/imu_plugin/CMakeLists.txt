cmake_minimum_required(VERSION 3.16)
project(imu_plugin)

# Busca paquetes de Ignition Gazebo (Fortress)
find_package(ignition-gazebo7 REQUIRED COMPONENTS core all)
find_package(ignition-math7 REQUIRED)
find_package(ignition-msgs9 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Incluye directorios
include_directories(
  include
  ${IGNITION-GAZEBO_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

# Define el plugin
add_library(imu_plugin SHARED
  src/imu_plugin.cpp
)

# Enlaza librerías
target_link_libraries(imu_plugin
  ${IGNITION-GAZEBO_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

# Instalación
install(TARGETS imu_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()