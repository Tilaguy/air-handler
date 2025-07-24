#!/bin/bash
# ========================
# ROS 2 + Gazebo Fortress Environment Setup
# ========================

# Set Gazebo Fortress version
export GZ_VERSION=fortress

# Ensure Gazebo plugin path includes ROS 2 Humble
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib"

# === MODEL & RESOURCE PATHS ===
MODEL_BASE_PATH="/ros2_ws/src"

# Initialize environment variables
GAZEBO_MODEL_PATHS=""
GZ_SIM_RESOURCE_PATHS=""

# Search for all directories containing a model.config
for dir in $(find $MODEL_BASE_PATH -type f -name "model.config" -exec dirname {} \;); do
    parent_dir=$(dirname "$dir")
    GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS}:${parent_dir}"
    GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS}:${dir}"
done

# Add default Gazebo Fortress models and resources
GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS}:/usr/share/gz-gui-${GZ_VERSION}/models:/usr/share/gz-sim-${GZ_VERSION}/worlds"
GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS}:/usr/share/gz-sim-${GZ_VERSION}"

# Remove leading colon
GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS#:}"
GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS#:}"

# Export variables
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATHS}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATHS}"

# === Source ROS 2 and workspace ===
source /opt/ros/humble/setup.bash
if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source /usr/share/gazebo/setup.sh
fi
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Make sure scripts are executable
chmod +x /ros2_ws/run_sim.sh 2>/dev/null || true
chmod +x /ros2_ws/tools/*.py 2>/dev/null || true
