air_handler_sim/
├── sensors/
│   ├── imu_sensor/
│   │   ├── urdf/            ← URDF o SDF del sensor
│   │   ├── config/          ← Ruido, filtros, parámetros
│   │   └── launch/          ← Launch individual del sensor
│   ├── camera_sensor/
│   ├── lidar_sensor/
│   └── ...
├── robot_description/       ← Base del Air Handler (estructura física)
├── robot_bringup/           ← Launch principal (usa include de sensores)
├── worlds/                  ← Mundos Gazebo
└── rviz/                    ← Visualización

