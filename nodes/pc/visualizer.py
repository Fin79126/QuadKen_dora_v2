"""
PC Rerun Visualizer Node
Subscribes to all robot dataflow streams and visualizes them in Rerun:
  - camera/image: 2D camera feed
  - bno_data: 3D body orientation and gyro/accel time-series
  - control_cmd: Command velocities and posture setpoints
  - compute_status: State machine and gait phase
  - esp_status: TCP connection states and latency for ESP1 & ESP2
  - esp_telemetry: UDP actuator feedback and telemetry
"""

import os
import sys
import time
import json
import socket
import cv2
import numpy as np
import pyarrow as pa
import rerun as rr
from dora import Node

# Rerun archetype compatibility (Rerun 0.20+ uses Scalars instead of Scalar)
rr_Scalar = getattr(rr, "Scalars", getattr(rr, "Scalar", None))

_last_viewer_check = 0.0
_viewer_alive = True


def check_viewer_alive() -> bool:
    """Check if Rerun Viewer gRPC server is responsive."""
    global _last_viewer_check, _viewer_alive
    now = time.time()
    if now - _last_viewer_check < 0.5:
        return _viewer_alive
    _last_viewer_check = now
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.02)
        res = s.connect_ex(("127.0.0.1", 9876))
        s.close()
        _viewer_alive = (res == 0)
    except Exception:
        _viewer_alive = False
    return _viewer_alive


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Convert Euler angles (degrees) to quaternion [x, y, z, w]."""
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [float(qx), float(qy), float(qz), float(qw)]


def main():
    # Initialize Rerun first so that the GUI viewer is ready BEFORE connecting to Dora
    rr.init("QuadKen_Dora_Telemetry", spawn=True)
    try:
        rr.unregister_shutdown()
    except Exception:
        pass
    print("[Visualizer] Rerun Viewer initialized.")

    node = Node()

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[Visualizer] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                if not check_viewer_alive():
                    continue

                input_id = event["id"]
                raw_value = event["value"]
                # Debug print for first few events of each type
                if not hasattr(main, "_seen_inputs"):
                    main._seen_inputs = set()
                if input_id not in main._seen_inputs:
                    main._seen_inputs.add(input_id)
                    print(f"[Visualizer] First input received for: {input_id}")

                # Handle Camera Images
                if input_id == "image":
                    try:
                        # Arrow array containing JPEG encoded bytes or raw array
                        img_data = raw_value.to_pylist()[0]
                        if isinstance(img_data, bytes):
                            # Decode JPEG
                            nparr = np.frombuffer(img_data, np.uint8)
                            frame_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            if frame_bgr is not None:
                                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                                rr.log("camera/feed", rr.Image(frame_rgb))
                        elif hasattr(raw_value, "to_numpy"):
                            np_img = raw_value.to_numpy()
                            rr.log("camera/feed", rr.Image(np_img))
                    except Exception as e:
                        print(f"[Visualizer] Camera log error: {e}")

                # Handle BNO IMU Data
                elif input_id == "bno_data":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        data = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        roll = float(data.get("roll", 0.0))
                        pitch = float(data.get("pitch", 0.0))
                        yaw = float(data.get("yaw", 0.0))
                        gyro = data.get("gyro", [0.0, 0.0, 0.0])
                        accel = data.get("accel", [0.0, 0.0, 9.81])

                        # 3D Orientation transform
                        quat_xyzw = euler_to_quaternion(roll, pitch, yaw)
                        rr.log(
                            "world/robot_base",
                            rr.Transform3D(
                                rotation=rr.Quaternion(xyzw=quat_xyzw),
                                translation=[0.0, 0.0, 0.25],
                            ),
                        )

                        # Time-series plots
                        rr.log("imu/orientation/roll", rr_Scalar(roll))
                        rr.log("imu/orientation/pitch", rr_Scalar(pitch))
                        rr.log("imu/orientation/yaw", rr_Scalar(yaw))

                        rr.log("imu/gyro/x", rr_Scalar(float(gyro[0])))
                        rr.log("imu/gyro/y", rr_Scalar(float(gyro[1])))
                        rr.log("imu/gyro/z", rr_Scalar(float(gyro[2])))

                        rr.log("imu/accel/x", rr_Scalar(float(accel[0])))
                        rr.log("imu/accel/y", rr_Scalar(float(accel[1])))
                        rr.log("imu/accel/z", rr_Scalar(float(accel[2])))
                    except Exception as e:
                        print(f"[Visualizer] BNO log error: {e}")

                # Handle Controller Commands
                elif input_id == "control_cmd":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        cmd = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        rr.log("control/cmd_vel/vx", rr_Scalar(float(cmd.get("vx", 0.0))))
                        rr.log("control/cmd_vel/vy", rr_Scalar(float(cmd.get("vy", 0.0))))
                        rr.log("control/cmd_vel/vyaw", rr_Scalar(float(cmd.get("vyaw", 0.0))))
                        rr.log("control/gait_mode", rr_Scalar(int(cmd.get("gait_mode", 0))))
                        rr.log("control/e_stop", rr_Scalar(1.0 if cmd.get("e_stop", False) else 0.0))

                        # Log raw joystick axes for hardware/stick testing
                        raw_axes = cmd.get("raw_axes", [])
                        for i, val in enumerate(raw_axes):
                            rr.log(f"joystick/raw_axis_{i}", rr_Scalar(float(val)))
                    except Exception as e:
                        print(f"[Visualizer] Controller log error: {e}")

                # Handle ESP Status (TCP connection states & Latency)
                elif input_id == "esp_status":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        status = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        esp1_conn = 1.0 if status.get("esp1", {}).get("connected", False) else 0.0
                        esp2_conn = 1.0 if status.get("esp2", {}).get("connected", False) else 0.0
                        esp1_rtt = float(status.get("esp1", {}).get("latency_ms", 0.0))
                        esp2_rtt = float(status.get("esp2", {}).get("latency_ms", 0.0))

                        rr.log("tcp_status/esp1/connected", rr_Scalar(esp1_conn))
                        rr.log("tcp_status/esp2/connected", rr_Scalar(esp2_conn))
                        rr.log("tcp_status/esp1/latency_ms", rr_Scalar(esp1_rtt))
                        rr.log("tcp_status/esp2/latency_ms", rr_Scalar(esp2_rtt))
                        rr.log("status_text/esp1", rr.TextLog(f"ESP1: {status.get('esp1', {}).get('state', 'UNKNOWN')}"))
                        rr.log("status_text/esp2", rr.TextLog(f"ESP2: {status.get('esp2', {}).get('state', 'UNKNOWN')}"))
                    except Exception as e:
                        print(f"[Visualizer] ESP status log error: {e}")

                # Handle ESP Telemetry (UDP sensor feedback)
                elif input_id == "esp_telemetry":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        telemetry = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        if "esp1" in telemetry:
                            t1 = telemetry["esp1"]
                            rr.log("esp1/voltage", rr_Scalar(float(t1.get("voltage", 12.0))))
                            rr.log("esp1/current", rr_Scalar(float(t1.get("current", 0.5))))
                        if "esp2" in telemetry:
                            t2 = telemetry["esp2"]
                            rr.log("esp2/voltage", rr_Scalar(float(t2.get("voltage", 12.0))))
                            rr.log("esp2/current", rr_Scalar(float(t2.get("current", 0.5))))
                    except Exception as e:
                        print(f"[Visualizer] ESP telemetry log error: {e}")

                # Handle Compute Status
                elif input_id == "compute_status":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        c_status = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))
                        rr.log("status_text/robot_state", rr.TextLog(f"State: {c_status.get('robot_state', 'UNKNOWN')}"))
                        rr.log("compute/dt_ms", rr_Scalar(float(c_status.get("dt_ms", 0.0))))
                    except Exception as e:
                        print(f"[Visualizer] Compute status log error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        os._exit(0)


if __name__ == "__main__":
    main()
