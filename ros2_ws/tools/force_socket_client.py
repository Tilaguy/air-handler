#!/usr/bin/env python3
import socket
import os
import struct
import time
import numpy as np

SOCKET_PATH = "/tmp/force_socket"

client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

print(f"[INFO] Connecting to socket: {SOCKET_PATH}")
while True:
    print('hola1')
    try:
        client_socket.connect(SOCKET_PATH)
        print("[INFO] Connected to force sensor socket.")
        break
    except socket.error:
        print(f"[INFO] Waiting for socket {SOCKET_PATH}...")
        time.sleep(1)

# Parámetros del sensor real
I2C_REQUEST_BYTE = 0x28
DIGITAL_MIN = 1638
DIGITAL_MAX = 14745
FORCE_MIN = 0.0  # N
FORCE_MAX = 15.0  # N

def digital_to_force(digital_output):
    if digital_output < DIGITAL_MIN:
        digital_output = DIGITAL_MIN
    elif digital_output > DIGITAL_MAX:
        digital_output = DIGITAL_MAX
    return FORCE_MIN + (digital_output - DIGITAL_MIN) * (FORCE_MAX - FORCE_MIN) / (DIGITAL_MAX - DIGITAL_MIN)

def decode_force_sample(raw_value):
    # Extraer status (bits 15 y 14)
    status = (raw_value & 0xC000) >> 14
    
    digital_data = 0
    force = np.NAN
    if status != 0:
        print("[INFO] Waiting for socket {SOCKET_PATH}...")
    else:
        # Extraer datos (14 bits)
        digital_data = raw_value & 0x3FFF

        # Convertir a Newtons si dentro del rango válido
        force = digital_to_force(digital_data)

    return status, digital_data, force

try:
    print('hola')
    while True:
        client_socket.send(bytes([I2C_REQUEST_BYTE, 0x01]))
        
        data = client_socket.recv(2)
        if len(data) != 2:
            print("[WARN] Incomplete data")
            continue

        print(data)
        raw_value = struct.unpack(">H", data)[0]  # Big-endian
        status, digital, force = decode_force_sample(raw_value)

        if force is not None:
            print(f"[INFO] Status: {status}, Digital: {digital}, Force: {force:.3f} N")
        else:
            print(f"[WARN] Digital value out of range: {digital}, Status: {status}")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("[INFO] Interrupted by user.")
finally:
    client_socket.close()
