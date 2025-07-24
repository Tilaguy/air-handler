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

---
---
---
---

# 🌐 Proyecto de Simulación Realista de Sensores en Gazebo + ROS 2

Hola ChatGPT, retomamos este proyecto desde cero en un nuevo hilo para continuar organizadamente. A continuación te dejo un resumen completo del estado actual, incluyendo infraestructura, sensores implementados, y lo que viene. Más abajo adjuntaré un PDF con documentación complementaria hecha en Notion.

---

## 🎯 Objetivo General

Desarrollar un entorno de simulación realista en **ROS 2 Humble + Gazebo Fortress (6.17.0)**, donde se simulen sensores reales con errores físicos y se comuniquen como en la vida real (p. ej. usando sockets que emulan protocolos como I2C, SPI, CAN, etc).

---

## ✅ Sensores Simulados

### 1. IMU (LSM6DSOX)

- Publicación por **socket UNIX** con formato binario:
  - Cabecera (0xA5), 6 valores `int16_t`, y 1 byte de checksum (XOR).
- Codificación:
  - Aceleraciones en m/s².
  - Giroscopio en rad/seg.
- Cliente en Python:
  - Decodifica el buffer, grafica con `matplotlib`, guarda CSV.
- **Modelo de error simulado:**
  - Ruido gaussiano
  - Clipping
  - Drift
  - Hysteresis
  - Cuantización

---

### 2. Sensor de Fuerza (Honeywell FMA Series)

- Mide solo **magnitud de fuerza perpendicular (Z)**.
- Codificación:
  - Basada en protocolo I2C del datasheet: 2 bits de estado + 14 bits de datos (2 bytes totales).
- Cliente en Python (en construcción):
  - Envia solcitud de datos mediante la direccion I2C del sensor (0x28) y un bit al final en 1 para simular la solicitud al sensor.
  - Decodifica, aplica escala, muestra fuerza en tiempo real.
- Simulación incluye:
  - Ruido, clipping, histéresis, cuantización, y drift.

---

## 🧰 Infraestructura

### Estructura de carpetas

```bash
/ros2_ws/src/
│
├── sensor_sim/
│   ├── worlds/           # Archivos .world (uno por sensor y uno global)
│   ├── launch/           # Launchers individuales para cada sensor
│   └── all_sensors.world # Archivo global que se actualiza automáticamente
│
├── imu_plugin/           # Plugin ROS 2 en C++ para la IMU
├── force_plugin/         # Plugin ROS 2 en C++ para el sensor de fuerza
└── ...
```

### Scripts de automatización

- `create_sensor_pkg.sh`:
  - Crea toda la estructura base de un sensor.
  - Reemplaza tokens como `__SENSOR_NAME__`, `__SOCKET_NAME__`, etc.
  - Agrega el sensor al archivo `all_sensors.world`.

## 🧪 Temas en desarrollo o por implementar

- Mejorar modelos 3D:
  - Mesh .dae optimizado desde FreeCAD.
- Codificar nuevos sensores (altímetro, flujo óptico, radar).
- Validar comunicaciones bajo errores o desconexiones.
- Crear pruebas unitarias para cada sensor que se simule.

## 📎 Documentación adicional

En el siguiente mensaje voy a adjuntar un PDF exportado desde Notion donde detallo más sobre la arquitectura, decisiones técnicas, motivación, y pruebas realizadas.
