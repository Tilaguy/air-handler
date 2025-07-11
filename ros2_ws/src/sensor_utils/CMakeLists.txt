cmake_minimum_required(VERSION 3.8)
project(sensor_utils)

find_package(ament_cmake REQUIRED)

# Opcional si usas archivos fuente .cpp
file(GLOB SOURCES "src/*.cpp")

add_library(sensor_utils ${SOURCES})
target_include_directories(sensor_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS sensor_utils
  EXPORT export_sensor_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_sensor_utils)
ament_package()
