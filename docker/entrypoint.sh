#!/bin/bash
set -e

# Build workspace if needed
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
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=all
# NOTE: xhost +local:root should ideally be run on the host, not inside the container.
xhost +local:root 2>/dev/null || true

# Add ros2_env sourcing to bashrc only if not already present
if ! grep -Fxq "source /ros2_env.sh" ~/.bashrc; then
    echo "source /ros2_env.sh" >> ~/.bashrc
fi

# Source environment now
source /ros2_env.sh

# Execute CMD
exec "$@"
