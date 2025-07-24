#!/bin/bash
# ========================
# ROS 2 + Gazebo Fortress Environment Setup
# ========================

# Set Gazebo Fortress version
export GZ_VERSION=fortress

# === MODEL & RESOURCE PATHS ===
MODEL_BASE_PATH="/ros2_ws/src"

# Initialize environment variables
GZ_SIM_RESOURCE_PATHS=""

# Search for all directories containing a model.config
for dir in $(find $MODEL_BASE_PATH -type f -name "model.config" -exec dirname {} \;); do
    parent_dir=$(dirname "$dir")
    GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS}:${parent_dir}"
done

# Add default Gazebo Fortress resources
GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS}:/usr/share/gz-sim-${GZ_VERSION}"

# Remove leading colon
GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS#:}"

# Export variables
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATHS}"

# === Source ROS 2 and workspace ===
source /opt/ros/humble/setup.bash

# Source Gazebo Fortress setup
if [ -f "/usr/share/gz/gz-tools/setup.sh" ]; then
    source /usr/share/gz/gz-tools/setup.sh
fi
if [ -f "/usr/share/gz/gz-${GZ_VERSION}/setup.sh" ]; then
    source /usr/share/gz/gz-${GZ_VERSION}/setup.sh
fi

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Make sure scripts are executable
chmod +x /ros2_ws/run_sim.sh 2>/dev/null || true
chmod +x /ros2_ws/tools/*.py 2>/dev/null || true
