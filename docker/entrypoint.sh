#!/bin/bash
set -e

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

echo "source /ros2_env.sh" >> ~/.bashrc

exec "$@"

