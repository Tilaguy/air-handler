# Repository organization
```
.
├── docker/
│   ├── docker-compose.yml
│   ├── Dockerfile
│   └── entrypoint.sh
└── ros2_ws/
    ├── build/
    ├── install/
    ├── log/
    └── src/
        └── sensor_plugins/
            ├── CMakeLists.txt
            ├── package.xml
            ├── include/
            │   └── sensor_plugins/
            │       ├── imu_plugin.hpp
            │       └── lidar_plugin.hpp
            ├── src/
            │   ├── imu_plugin.cpp
            │   └── lidar_plugin.cpp
            ├── models/
            │   └── imu/
            │       ├── imu_sensor.sdf
            │       └── config/
            │           └── imu_params.yaml
            └── launch/
                └── test_imu.launch.py
```
# Software versions
| Software | Version  |
| :------- | :------: |
| Ubuntu   | 24.04    |
| ROS2     | Rolling  |
| Gazebo   | Harmonic |
| Python   |  |
| C++      |  |


