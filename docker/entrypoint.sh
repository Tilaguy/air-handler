#!/bin/bash

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace if src/ exists
if [ -d "/ros2_ws/src" ]; then
  echo "Building ROS 2 workspace..."
  cd /ros2_ws && colcon build --symlink-install
  source install/setup.bash
fi

# Launch a shell (or your preferred command)
exec bash