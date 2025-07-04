# Prototype 2 - ROS 2 Rolling + Gazebo Harmonic Plugin Development

This repository contains a complete development environment for building **C++ sensor plugins** for **Gazebo Harmonic (GZ Sim 8)** integrated with **ROS 2 Rolling**.  
The goal is to simulate sensors as closely as possible to the real hardware for early validation in simulation.

---

## 🗂️ Project Structure

```
├── config/
│ └── common_params.yaml # Shared configuration
├── docker/
│ ├── Dockerfile # Docker image with ROS 2 + Gazebo Harmonic
│ ├── docker-compose.yml # Container configuration
│ ├── entrypoint.sh # Builds and sources ROS 2 workspace
│ └── .env # DISPLAY config for GUI
├── ros2_ws/
│ ├── build/ # Build artifacts (ignored)
│ ├── install/ # Installed packages (ignored)
│ ├── log/ # Colcon logs (ignored)
│ └── src/
│ └── sensor_plugins/ # C++ Gazebo plugins package
│ ├── CMakeLists.txt
│ ├── package.xml
│ ├── include/
│ │ └── sensor_plugins/
│ │ ├── imu_plugin.hpp
│ │ └── lidar_plugin.hpp
│ ├── src/
│ │ ├── imu_plugin.cpp
│ │ └── lidar_plugin.cpp
│ ├── models/
│ │ └── imu/
│ │ ├── imu_sensor.sdf
│ │ └── config/
│ │ └── imu_params.yaml
│ └── launch/
│ └── test_imu.launch.py
```

## 💻 Software Versions

| Software | Version            |
| -------- | :----------------: |
| Ubuntu   | 22.04 (Jammy)      |
| ROS 2    | Rolling            |
| Gazebo   | Harmonic (GZ Sim 8)|
| Python   | 3.10               |
| C++      | 17 (default)       |

---

## 🚀 Quick Start

### 1. Build the Docker image

```bash
cd docker
docker compose build
```

2. **Access the development container**

```bash
docker exec -it sensor_plugins_dev bash
```

3. **Rebuild the workspace manually (if needed)**

```bash
cd /ros2_ws
colcon build --symlink-install
```

4. **Run Gazebo**

```bash
gz sim
```

# 🧪 Testing Plugins

If a plugin and launch file are implemented:

```bash
ros2 launch sensor_plugins test_imu.launch.py
```

To manually run Gazebo:

```bash
gz sim
```

Or load a specific SDF model:

```bash
gz sim ~/ros2_ws/src/sensor_plugins/models/imu/imu_sensor.sdf
```

# 🛠️ Useful Commands

Inside the container:

```bash
# Rebuild the workspace manually
cd /ros2_ws
colcon build --symlink-install

# Source environment
source install/setup.bash
```
