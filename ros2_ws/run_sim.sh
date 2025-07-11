#!/bin/bash

# ------------------------------
# List available plugins
# ------------------------------
if [ "$1" == "--list" ]; then
  echo "[INFO] Available plugins:"
  echo "all"
  find /ros2_ws/src -maxdepth 1 -type d -name "*_plugin" | sed 's|.*/||' | sed 's/_plugin//'
  exit 0
fi

# ------------------------------
# Parameters with default values
# ------------------------------
PLUGIN=${1:-"all"}
SIM_PKG=${2:-"sensor_sim"}
PLUGIN_PKG="$PLUGIN"_plugin
LAUNCH_FILE="$PLUGIN"_test.launch.py

echo "[INFO] Killing any running Gazebo processes..."
pkill -f gz > /dev/null 2>&1 && echo "[INFO] Gazebo stopped." || echo "[INFO] No Gazebo process found."

# ------------------------------
# Validate plugin excistence
# ------------------------------
if [ "$PLUGIN" != "all" ] && [ ! -d "/ros2_ws/src/${PLUGIN}_plugin" ]; then
  echo "[ERROR] Plugin package '${PLUGIN}_plugin' not found in src/"
  echo "[TIP] List available plugins with: ./run_sim.sh --list"
  exit 1
fi

echo "[INFO] Building workspace..."

cd /ros2_ws || exit 1

# Build entire workspace or just one package
if [ "$PLUGIN" == "all" ]; then
  echo "[INFO] Building all packages..."
  colcon build --symlink-install
else
  echo "[INFO] Building only package: $PLUGIN_PKG"
  colcon build --packages-select "$PLUGIN_PKG" --symlink-install
fi

# Check if build was successful
if [ $? -ne 0 ]; then
  echo "[ERROR] Build failed."
  exit 1
fi

echo "[INFO] Sourcing environment..."
source /ros2_ws/install/setup.bash

# ------------------------------
# Setup GAZEBO_MODEL_PATH from all *_plugin/model directories
# ------------------------------
GAZEBO_MODEL_PATH_DEFAULT="/usr/share/gazebo-11/models"
MODEL_PATHS=$(find /ros2_ws/src -maxdepth 3 -type d -name models)
GAZEBO_MODEL_PATH_COMBINED=$(echo "$MODEL_PATHS" | tr '\n' ':')

export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH_COMBINED:$GAZEBO_MODEL_PATH_DEFAULT"
echo "[INFO] GAZEBO_MODEL_PATH set to: $GAZEBO_MODEL_PATH"

# Check if the launch file exists in the specified package
LAUNCH_PATH="/ros2_ws/src/$SIM_PKG/launch/$LAUNCH_FILE"
if [ ! -f "$LAUNCH_PATH" ] && [ "$PLUGIN" != "all" ]; then
  echo "[ERROR] Launch file '$LAUNCH_PATH' not found."
  echo "[TIP] You can list plugins with: ./run_sim.sh --list"
  exit 1
fi

echo "[INFO] Launching simulation from package '$SIM_PKG' with launch file '$LAUNCH_FILE'..."
ros2 launch "$SIM_PKG" "$LAUNCH_FILE"
