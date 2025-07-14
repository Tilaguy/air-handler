#!/usr/bin/env python3
import socket
import os
import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

SOCKET_PATH = "/tmp/imu_socket"

# Config CSV log
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
log_dir = "/ros2_ws/logs"
csv_file_path = f"{log_dir}/imu_data_{timestamp}.csv"
os.makedirs(log_dir, exist_ok=True)

# Socket
client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

while not os.path.exists(SOCKET_PATH):
    print(f"[INFO] Waiting for socket {SOCKET_PATH}...")
    time.sleep(1)

client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

try:
    client_socket.connect(SOCKET_PATH)
    print("[INFO] Connected to IMU socket.")
except socket.error as e:
    print(f"[ERROR] Could not connect to socket: {e}")
    exit(1)
    
# Use file-like object for line-based reading
sock_file = client_socket.makefile('r')  # text mode, line-buffered

# Buffers
max_points = 200
acc_data = {"x": [], "y": [], "z": []}
gyro_data = {"x": [], "y": [], "z": []}

# Matplotlib config
fig, axs = plt.subplots(2, 1, figsize=(10, 6))
plt.subplots_adjust(hspace=0.4)
axs[0].set_title("Linear Acceleration [m/s²]")
axs[1].set_title("Angular Velocity [rad/s]")
# Inicializar curvas con líneas vacías
lines_acc = {k: axs[0].plot([], [], label=f"acc_{k}")[0] for k in ["x", "y", "z"]}
lines_gyro = {k: axs[1].plot([], [], label=f"gyro_{k}")[0] for k in ["x", "y", "z"]}

# Set limits y leyendas
for ax in axs:
    ax.set_xlim(0, max_points)
    ax.set_ylim(-10, 10)  # Ajusta según tus rangos esperados
    ax.legend()
    ax.grid()

def animate(i):
    data = sock_file.readline()
    if not data:
        return

    try:
        ax_val, ay_val, az_val, gx_val, gy_val, gz_val = map(float, data.strip().split(","))

        # Actualiza buffers
        for k, v in zip(["x", "y", "z"], [ax_val, ay_val, az_val]):
            acc_data[k].append(v)
            acc_data[k] = acc_data[k][-max_points:]
        for k, v in zip(["x", "y", "z"], [gx_val, gy_val, gz_val]):
            gyro_data[k].append(v)
            gyro_data[k] = gyro_data[k][-max_points:]

        x_range = list(range(len(acc_data["x"])))

        # Actualiza líneas sin limpiar
        for k in ["x", "y", "z"]:
            lines_acc[k].set_data(x_range, acc_data[k])
            lines_gyro[k].set_data(x_range, gyro_data[k])

        for ax in axs:
            ax.set_xlim(0, max_points)
            # Opcional: autoajustar límites Y si tus valores varían mucho
            # ax.relim()
            # ax.autoscale_view()

        # Guardar CSV
        with open(csv_file_path, "a") as f:
            f.write(data)

    except Exception as e:
        print("[ERROR]", e)

# Setup: establecer límites al inicio
for ax in axs:
    ax.set_xlim(0, max_points)
    ax.set_ylim(-10, 10)  # Puedes ajustar a valores esperados
    ax.grid()

ani = animation.FuncAnimation(fig, animate, interval=50)

print(f"[INFO] Logging to {csv_file_path}")
print("[INFO] Press Ctrl+C to stop")

try:
    plt.show()
except KeyboardInterrupt:
    print("[INFO] Interrupted by user.")
finally:
    client_socket.close()
