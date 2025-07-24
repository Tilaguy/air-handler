import os
import socket
import struct
import time
import pytest
import rclpy
from gazebo_msgs.srv import ApplyBodyWrench
import matplotlib.pyplot as plt


SOCKET_PATH = "/tmp/imu_socket"
FORCE_TOPIC = "/apply_force"  # Assumes a test node is subscribed to this

@pytest.fixture(scope="module")
def imu_socket():
    """Connect to the IMU UNIX socket."""
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(2.0)
    try:
        client.connect(SOCKET_PATH)
    except Exception as e:
        pytest.fail(f"[✘] IMU socket unavailable: {e}")
    yield client
    client.close()

def read_packet(client_socket):
    sync_byte = 0xA5
    while True:
        head = client_socket.recv(1)
        if not head:
            continue
        if head[0] == sync_byte:
            rest = client_socket.recv(13)
            if len(rest) != 13:
                continue
            packet = bytes([sync_byte]) + rest
            print(packet)
            return packet

def receive_imu_packet(sock):
    """Receive and decode an IMU data packet."""
    data = read_packet(sock)
    print(len(data))
    if len(data) != 14:
        pytest.fail(f"[✘] Invalid packet length: {len(data)}")

    if data[0] != 0xA5:
        pytest.fail(f"[✘] Invalid header byte: {data[0]:#x}")

    checksum = 0
    for i in range(13):
        checksum ^= data[i]
    if data[13] != checksum:
        pytest.fail(f"[✘] Invalid checksum: {data[13]} vs expected {checksum}")

    acc_x = struct.unpack('<h', data[1:3])[0] * 0.01
    acc_y = struct.unpack('<h', data[3:5])[0] * 0.01
    acc_z = struct.unpack('<h', data[5:7])[0] * 0.01
    gyr_x = struct.unpack('<h', data[7:9])[0] * 0.001
    gyr_y = struct.unpack('<h', data[9:11])[0] * 0.001
    gyr_z = struct.unpack('<h', data[11:13])[0] * 0.001

    return {
        "acc": (acc_x, acc_y, acc_z),
        "gyro": (gyr_x, gyr_y, gyr_z)
    }


def test_imu_static_values(imu_socket):
    """Validate IMU output in static condition."""
    print("\n[INFO] Validating static sensor readings...")
    values = receive_imu_packet(imu_socket)

    acc = values["acc"]
    gyro = values["gyro"]

    for i, a in enumerate(acc):
        assert -20 < a < 20, f"[✘] acc[{i}] out of range: {a:.3f}"
    for i, g in enumerate(gyro):
        assert -10 < g < 10, f"[✘] gyro[{i}] out of range: {g:.3f}"

    print("[✔] Static values within expected range.")


def test_imu_reacts_to_force(imu_socket):
    """Apply wrench via Gazebo service and validate IMU detects it."""

    print("\n[INFO] Applying force using /gazebo/apply_body_wrench service...")

    rclpy.init()
    node = rclpy.create_node("imu_force_test_node")

    client = node.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

    if not client.wait_for_service(timeout_sec=5.0):
        pytest.fail("[✘] Service /gazebo/apply_body_wrench not available")

    request = ApplyBodyWrench.Request()
    request.body_name = "imu_box::imu_body"
    request.reference_frame = "imu_box::imu_body"
    request.wrench.force.z = 5.0
    request.duration.sec = 0
    request.duration.nanosec = int(0.5 * 1e9)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        pytest.fail("[✘] Failed to apply wrench")
    else:
        print("[✔] Wrench applied to IMU box")

    node.destroy_node()
    rclpy.shutdown()

    # Collect IMU data for 2 seconds after applying the force
    acc_z_values = []
    timestamps = []
    start_time = time.time()

    while time.time() - start_time < 2.0:
        values = receive_imu_packet(imu_socket)
        acc_z = values["acc"][2]
        acc_z_values.append(acc_z)
        timestamps.append(time.time() - start_time)
        time.sleep(0.05)  # 20 Hz sample rate

    # Validate if some value exceeded threshold (indicating movement)
    if not any(abs(v) > 0.5 for v in acc_z_values):
        pytest.fail("[✘] No noticeable response in IMU Z axis")
    else:
        print(f"[✔] IMU responded to stimulus. Peak acc_z = {max(acc_z_values):.3f}")

    # Plotting
    plt.figure(figsize=(8, 4))
    plt.plot(timestamps, acc_z_values, label="acc_z (m/s²)", color='blue')
    plt.axhline(y=0.5, color='red', linestyle='--', label="Threshold")
    plt.title("IMU Z-axis acceleration after stimulus")
    plt.xlabel("Time (s)")
    plt.ylabel("acc_z (m/s²)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("imu_response_plot.png")
    print("[✔] Plot saved as imu_response_plot.png")