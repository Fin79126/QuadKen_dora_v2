"""
Raspberry Pi BNO IMU Node (QuadKen Underwater AUV)
Interfaces with BNO055 / BNO085 orientation sensor.
Provides realistic simulated underwater motion when hardware is not detected.
Publishes:
  - bno_data: Roll, Pitch, Yaw, Gyro (deg/s), Accel (m/s^2), and Depth (m).
"""

import sys
import time
import json
import numpy as np
import pyarrow as pa
from dora import Node

# Try importing hardware libraries if running on real Raspberry Pi
try:
    import board
    import busio
    import adafruit_bno055
    HAS_HW = True
except (ImportError, NotImplementedError):
    HAS_HW = False


def init_hardware_bno():
    if not HAS_HW:
        return None
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        print("[BNO] BNO055 Hardware I2C initialized successfully.")
        return sensor
    except Exception as e:
        print(f"[BNO] Hardware init failed ({e}). Using simulated underwater BNO IMU.")
        return None


def main():
    node = Node()
    bno_sensor = init_hardware_bno()

    seq = 0
    start_time = time.time()

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[BNO] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                seq += 1
                now = time.time()
                t = now - start_time

                if bno_sensor is not None:
                    try:
                        euler = bno_sensor.euler
                        gyro = bno_sensor.gyro
                        accel = bno_sensor.acceleration
                        quat = bno_sensor.quaternion

                        yaw = float(euler[0]) if euler[0] is not None else 0.0
                        roll = float(euler[1]) if euler[1] is not None else 0.0
                        pitch = float(euler[2]) if euler[2] is not None else 0.0

                        gyro_data = [float(g) if g is not None else 0.0 for g in gyro]
                        accel_data = [float(a) if a is not None else 0.0 for a in accel]
                        depth_m = 1.5
                    except Exception:
                        bno_sensor = None
                        yaw, roll, pitch = 0.0, 0.0, 0.0
                        gyro_data = [0.0, 0.0, 0.0]
                        accel_data = [0.0, 0.0, 9.81]
                        depth_m = 1.5
                else:
                    # Simulated smooth underwater hydrodynamic motion
                    # Natural pitch oscillation and gentle roll swaying from water current
                    pitch = float(3.0 * np.sin(0.4 * t) - 1.0 * np.sin(0.8 * t))
                    roll = float(1.5 * np.cos(0.5 * t))
                    yaw = float((t * 5.0) % 360.0 - 180.0)

                    gyro_data = [
                        round(float(0.8 * np.sin(0.5 * t)), 2),
                        round(float(1.2 * np.cos(0.4 * t)), 2),
                        round(float(5.0), 2),
                    ]
                    accel_data = [
                        round(float(0.2 * np.sin(0.3 * t)), 2),
                        round(float(0.1 * np.cos(0.4 * t)), 2),
                        round(float(9.81 + 0.1 * np.sin(0.2 * t)), 2),
                    ]
                    # Simulated depth oscillating around 1.8m
                    depth_m = round(float(1.8 + 0.3 * np.sin(0.2 * t)), 2)

                payload = {
                    "seq": seq,
                    "timestamp": now,
                    "roll": round(roll, 2),
                    "pitch": round(pitch, 2),
                    "yaw": round(yaw, 2),
                    "gyro": gyro_data,
                    "accel": accel_data,
                    "depth_m": depth_m,
                }

                node.send_output("bno_data", pa.array([json.dumps(payload).encode("utf-8")]))
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
