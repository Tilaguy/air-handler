# === EXPORT PATHS ===
# Base path where your models are located
MODEL_BASE_PATH="/ros2_ws/src"

# Initialize environment variable values
GAZEBO_MODEL_PATHS=""
GZ_SIM_RESOURCE_PATHS=""

# Search for all model directories containing a model.config file
for dir in $(find $MODEL_BASE_PATH -type f -name "model.config" -exec dirname {} \;); do
    parent_dir=$(dirname "$dir")
    GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS}:${parent_dir}"
    GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS}:${dir}"
done

# Add default Gazebo models directory
GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS}:/usr/share/gazebo-11/models"

GAZEBO_MODEL_PATHS="${GAZEBO_MODEL_PATHS:1}"
GZ_SIM_RESOURCE_PATHS="${GZ_SIM_RESOURCE_PATHS:1}"

# Export the environment variables
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATHS}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATHS}"

source /opt/ros/humble/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

chmod +x /ros2_ws/run_sim.sh
chmod +x /ros2_ws/tools/*.py