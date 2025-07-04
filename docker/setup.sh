#!/bin/bash

# Source ROS Rolling and Gazebo Harmonic
source /opt/ros/rolling/setup.bash
source /usr/share/gz/gz-sim8/setup.sh

# Source the workspace (if built)
if [ -f "$(pwd)/ros2_ws/install/setup.bash" ]; then
  source "$(pwd)/ros2_ws/install/setup.bash"
fi

# Export GZ version
export GZ_VERSION=harmonic

echo "Environment configured. Ready to develop!"
