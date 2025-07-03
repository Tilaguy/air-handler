#!/bin/bash

# === CONFIGURACIÓN ===
SENSOR_NAME=$1

if [ -z "$SENSOR_NAME" ]; then
  echo "❌ Uso: ./create_sensor_pkg.sh <nombre_sensor>"
  exit 1
fi

PACKAGE_NAME=${SENSOR_NAME}_sensor
WORKSPACE=~/prototype2/ros2_ws
SRC_DIR=$WORKSPACE/src/sensors/$PACKAGE_NAME

echo "📦 Creando paquete: $PACKAGE_NAME"

# === ESTRUCTURA DE CARPETAS ===
mkdir -p $SRC_DIR/{src,launch,config,urdf}

# === package.xml ===
cat <<EOF > $SRC_DIR/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>${PACKAGE_NAME}</name>
  <version>0.0.1</version>
  <description>Sensor package for ${SENSOR_NAME}</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# === CMakeLists.txt ===
cat <<EOF > $SRC_DIR/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(${PACKAGE_NAME})

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${SENSOR_NAME}_node src/${SENSOR_NAME}_node.cpp)

ament_target_dependencies(
  ${SENSOR_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  ${SENSOR_NAME}_node
  DESTINATION lib/\${PROJECT_NAME}
)

ament_package()
EOF

# === Código base (src) ===
cat <<EOF > $SRC_DIR/src/${SENSOR_NAME}_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ${SENSOR_NAME^}Node : public rclcpp::Node
{
public:
  ${SENSOR_NAME^}Node() : Node("${SENSOR_NAME}_node")
  {
    RCLCPP_INFO(this->get_logger(), "✅ ${SENSOR_NAME^} sensor node started!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${SENSOR_NAME^}Node>());
  rclcpp::shutdown();
  return 0;
}
EOF

echo "✅ Paquete $PACKAGE_NAME creado en: $SRC_DIR"
