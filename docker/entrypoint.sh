#!/bin/bash
set -e

# === EXPORT PATHS ===
export GAZEBO_MODEL_PATH=/ros2_ws/src/imu_plugin/models:/usr/share/gazebo-11/models
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/imu_plugin/models


# Build workspace only if 'install' folder is missing
if [ -d /ros2_ws/src ] && [ "$(ls -A /ros2_ws/src)" ]; then
  if [ ! -d /ros2_ws/install ]; then
    echo "[ENTRYPOINT] Building workspace (install/ not found)..."
    colcon build --symlink-install
  else
    echo "[ENTRYPOINT] Skipping build, install/ already exists."
  fi
fi

# GUI support
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
xhost +local:root 2>/dev/null || true

# Source ROS 2
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
# Source the setup if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
  grep -qxF 'source /ros2_ws/install/setup.bash' ~/.bashrc || echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc
fi

grep -qxF 'chmod +x /ros2_ws/run_sim.sh' ~/.bashrc || echo 'chmod +x /ros2_ws/run_sim.sh' >> ~/.bashrc
grep -qxF 'chmod +x /ros2_ws/tools/*.py' ~/.bashrc || echo 'chmod +x /ros2_ws/tools/*.py' >> ~/.bashrc

exec "$@"

