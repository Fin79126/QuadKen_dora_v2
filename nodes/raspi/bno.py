"""
Raspberry Pi BNO IMU Node
Interfaces with BNO055 / BNO085 orientation sensor.
Provides realistic simulated telemetry when hardware is not detected.
Publishes:
  - bno_data: Roll, Pitch, Yaw, Gyro (deg/s), Accel (m/s^2), and Quaternions.
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
        print(f"[BNO] Hardware init failed ({e}). Using simulated BNO IMU.")
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
                # Driven by timer tick (e.g. dora/timer/millis/20 = 50Hz)
                seq += 1
                now = time.time()
                t = now - start_time

                if bno_sensor is not None:
                    try:
                        euler = bno_sensor.euler or (0.0, 0.0, 0.0)
                        gyro = bno_sensor.gyro or (0.0, 0.0, 0.0)
                        accel = bno_sensor.linear_acceleration or (0.0, 0.0, 9.81)
                        quat = bno_sensor.quaternion or (1.0, 0.0, 0.0, 0.0)

                        yaw, roll, pitch = euler[0], euler[1], euler[2]
                        gx, gy, gz = gyro[0], gyro[1], gyro[2]
                        ax, ay, az = accel[0], accel[1], accel[2]
                        qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
                    except Exception:
                        # Sensor read glitch fallback
                        yaw, roll, pitch = 0.0, 0.0, 0.0
                        gx, gy, gz = 0.0, 0.0, 0.0
                        ax, ay, az = 0.0, 0.0, 9.81
                        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
                else:
                    # Simulated quadruped body sway and vibration
                    roll = 3.0 * np.sin(2.0 * np.pi * 0.8 * t)
                    pitch = 2.0 * np.cos(2.0 * np.pi * 0.8 * t)
                    yaw = (t * 5.0) % 360.0

                    gx = 0.8 * np.cos(2.0 * np.pi * 0.8 * t)
                    gy = -0.6 * np.sin(2.0 * np.pi * 0.8 * t)
                    gz = 0.1 * np.random.randn()

                    ax = 0.2 * np.sin(2.0 * np.pi * 1.6 * t) + 0.05 * np.random.randn()
                    ay = 0.2 * np.cos(2.0 * np.pi * 1.6 * t) + 0.05 * np.random.randn()
                    az = 9.81 + 0.5 * np.sin(2.0 * np.pi * 1.6 * t)

                    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

                payload = {
                    "seq": seq,
                    "timestamp": now,
                    "roll": round(float(roll), 2),
                    "pitch": round(float(pitch), 2),
                    "yaw": round(float(yaw), 2),
                    "gyro": [round(float(gx), 3), round(float(gy), 3), round(float(gz), 3)],
                    "accel": [round(float(ax), 3), round(float(ay), 3), round(float(az), 3)],
                    "quaternion": [round(float(qw), 4), round(float(qx), 4), round(float(qy), 4), round(float(qz), 4)],
                }

                payload_bytes = json.dumps(payload).encode("utf-8")
                node.send_output("bno_data", pa.array([payload_bytes]))
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
