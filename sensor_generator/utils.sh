#!/bin/bash

# --- Logging utilities ---
log_info() {
  echo -e "\e[32m[INFO]\e[0m $1"
}

log_warn() {
  echo -e "\e[33m[WARN]\e[0m $1"
}

log_error() {
  echo -e "\e[31m[ERROR]\e[0m $1"
}

# --- Input validation ---
validate_inputs() {
  if [ -z "$1" ]; then
    log_error "Usage: ./create_sensor_pkg.sh <sensor_name> [description]"
    exit 1
  fi
}

# --- Setup global vars from input ---
setup_variables() {
  SENSOR_NAME=$1
  SENSOR_DESCRIPTION=${2:-"Sensor plugin for ${SENSOR_NAME} simulation."}
  PKG_NAME="${SENSOR_NAME}_plugin"
  SOCKET_NAME="${SENSOR_NAME^}Plugin"  # Capitalize first letter
  WORKSPACE=~/prototype2/ros2_ws
  PKG_DIR=$WORKSPACE/src/$PKG_NAME
}

# --- Directory creation ---
create_directories() {
  mkdir -p "$PKG_DIR/include/${PKG_NAME}"
  mkdir -p "$PKG_DIR/src"
  mkdir -p "$PKG_DIR/models"
  mkdir -p "$PKG_DIR/models/${SENSOR_NAME}_box"
  mkdir -p "$PKG_DIR/models/${SENSOR_NAME}_box/meshes"
  mkdir -p "${WORKSPACE}/src/sensor_sim/launch"
  mkdir -p "${WORKSPACE}/src/sensor_sim/worlds"
}

# --- Template copy ---
copy_templates() {
  cp templates/CMakeLists.txt.in             "$PKG_DIR/CMakeLists.txt"
  cp templates/package.xml.in                "$PKG_DIR/package.xml"
  cp templates/plugin.cpp.in                 "$PKG_DIR/src/${PKG_NAME}.cpp"
  cp templates/plugin.hpp.in                 "$PKG_DIR/include/${PKG_NAME}/${PKG_NAME}.hpp"
  cp templates/model/model.sdf.in            "$PKG_DIR/models/model.sdf"
  cp templates/model/model.config.in         "$PKG_DIR/models/model.config"
  cp templates/launch/launch.launch.py.in    "${WORKSPACE}/src/sensor_sim/launch/${SENSOR_NAME}_test.launch.py"
  cp templates/world/test.world.in           "${WORKSPACE}/src/sensor_sim/worlds/${SENSOR_NAME}_test.world"
}

# --- Token replacement ---
replace_tokens() {
  find "$PKG_DIR" "${WORKSPACE}/src/sensor_sim/launch" "${WORKSPACE}/src/sensor_sim/worlds" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.xml" -o -name "*.sdf" -o -name "*.py" -o -name "*_test.world" -o -name "model.config" -o -name "CMakeLists.txt" \) | while read -r file; do
    sed -i \
      -e "s|__SENSOR_NAME__|${SENSOR_NAME}|g" \
      -e "s|__PKG_NAME__|${PKG_NAME}|g" \
      -e "s|__SOCKET_NAME__|${SOCKET_NAME}|g" \
      -e "s|__SENSOR_DESCRIPTION__|${SENSOR_DESCRIPTION}|g" \
      -e "s|__SENSOR_NAME_UPPER__|${PKG_NAME^^}|g" \
      "$file"
  done
}

add_sensor_to_world() {
  WORLD_FILE="${WORKSPACE}/src/sensor_sim/worlds/all_sensors.world"

  # Check if sensor is already in world file
  if grep -q "model://${SENSOR_NAME}_box" "$WORLD_FILE"; then
    log_warn "Sensor '${SENSOR_NAME}' already present in all_sensors.world"
    return
  fi

  # Insert the <include> inside the marker block
  awk -v sensor="$SENSOR_NAME" '
    /<!-- Begin import each sensor box -->/ {
      print
      print "    <include>"
      print "      <uri>model://"sensor"_box</uri>"
      print "    </include>"
      next
    }
    { print }
  ' "$WORLD_FILE" > "${WORLD_FILE}.tmp" && mv "${WORLD_FILE}.tmp" "$WORLD_FILE"

  log_info "Sensor '${SENSOR_NAME}' added to all_sensors.world"
}


# --- Final output message ---
final_message() {
  log_info "✅ Sensor plugin '$PKG_NAME' created!"
  echo "    cd ros2_ws && colcon build --packages-select $PKG_NAME"
}