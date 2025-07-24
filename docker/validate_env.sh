#!/bin/bash
echo "===== VALIDATING ROS 2 + Gazebo + GPU ====="

# --- ROS 2 Distro ---
ROS_DISTRO=$(printenv | grep ROS_DISTRO | cut -d'=' -f2)
if [ "$ROS_DISTRO" = "humble" ]; then
  echo "[OK] ROS 2 Distro: $ROS_DISTRO"
else
  echo "[FAIL] ROS 2 Distro not Humble (found: $ROS_DISTRO)"
fi

# --- Gazebo Fortress Version ---
if command -v ign &> /dev/null; then
  GZ_VER=$(ign gazebo --versions | head -n 1)
  if [[ $GZ_VER == 6* ]]; then
    echo "[OK] Gazebo Fortress detected (ign gazebo $GZ_VER)"
  else
    echo "[FAIL] Gazebo version unexpected: $GZ_VER (Fortress = 6.x)"
  fi
else
  echo "[FAIL] ign gazebo command not found"
fi

# --- Gazebo Classic ---
if command -v gazebo &> /dev/null; then
  echo "[FAIL] Gazebo Classic installed"
else
  echo "[OK] No Gazebo Classic detected"
fi

# --- ROS-GZ Bridge ---
if ros2 pkg list | grep -q ros_gz; then
  echo "[OK] ros_gz bridge packages installed"
else
  echo "[FAIL] ros_gz bridge not found"
fi

# --- GPU (OpenGL Renderer) ---
if command -v glxinfo &> /dev/null; then
  GPU_RENDERER=$(glxinfo | grep "OpenGL renderer" | awk -F': ' '{print $2}')
  if [[ "$GPU_RENDERER" == *"NVIDIA"* ]]; then
    echo "[OK] GPU Renderer: $GPU_RENDERER"
  else
    echo "[FAIL] Renderer is $GPU_RENDERER (likely software rendering)"
  fi
else
  echo "[FAIL] glxinfo not available"
fi

echo "===== VALIDATION COMPLETE ====="
