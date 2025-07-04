#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace if exists
if [ -f /ros2_ws/src/CMakeLists.txt ] || [ "$(ls -A /ros2_ws/src)" ]; then
  echo "[ENTRYPOINT] Building workspace..."
  colcon build --symlink-install
  source install/setup.bash
fi

# Source del workspace si ya existe
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source /root/ros2_ws/install/setup.bash
fi

# GUI support
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
xhost +local:root 2>/dev/null || true

exec "$@"
