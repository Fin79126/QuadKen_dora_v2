"""
PC Rerun Visualizer Node (QuadKen Underwater AUV)
Subscribes to all robot dataflow streams and visualizes them in Rerun:
  - image: 2D underwater forward camera feed
  - image_overhead: 2D third-person chase/overhead camera feed
  - bno_data: 3D body orientation and gyro/accel time-series
  - control_cmd: Command velocities, steering setpoints, ballast level
  - compute_status: AUV state, BLDC thrust, membrane leg deployment angles, ballast ratio
  - esp_status: TCP connection states and latency for ESP1 & ESP2
  - esp_telemetry: UDP sensor readings and telemetry from ESP1 & ESP2
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
    rr.init("QuadKen_Underwater_Telemetry", spawn=True)
    try:
        rr.unregister_shutdown()
    except Exception:
        pass
    print("[Visualizer] Rerun Viewer initialized for QuadKen Underwater AUV.")

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

                # 1. Handle Camera Images (Underwater Feed & Overhead Chase Feed)
                if input_id == "image":
                    try:
                        img_data = raw_value.to_pylist()[0]
                        if isinstance(img_data, bytes):
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

                elif input_id == "image_overhead":
                    try:
                        img_data = raw_value.to_pylist()[0]
                        if isinstance(img_data, bytes):
                            nparr = np.frombuffer(img_data, np.uint8)
                            frame_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            if frame_bgr is not None:
                                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                                rr.log("camera/overhead", rr.Image(frame_rgb))
                        elif hasattr(raw_value, "to_numpy"):
                            np_img = raw_value.to_numpy()
                            rr.log("camera/overhead", rr.Image(np_img))
                    except Exception as e:
                        print(f"[Visualizer] Overhead camera log error: {e}")

                # 2. Handle BNO IMU Data (Underwater Orientation)
                elif input_id == "bno_data":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        data = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        roll = float(data.get("roll", 0.0))
                        pitch = float(data.get("pitch", 0.0))
                        yaw = float(data.get("yaw", 0.0))
                        gyro = data.get("gyro", [0.0, 0.0, 0.0])
                        accel = data.get("accel", [0.0, 0.0, 9.81])

                        # 3D Orientation transform for cylindrical hull
                        quat_xyzw = euler_to_quaternion(roll, pitch, yaw)
                        rr.log(
                            "world/auv_hull",
                            rr.Transform3D(
                                rotation=rr.Quaternion(xyzw=quat_xyzw),
                                translation=[0.0, 0.0, -1.0],  # Underwater reference depth
                            ),
                        )

                        # Orientation & IMU plots
                        rr.log("imu/roll", rr_Scalar(roll))
                        rr.log("imu/pitch", rr_Scalar(pitch))
                        rr.log("imu/yaw", rr_Scalar(yaw))
                        rr.log("imu/gyro/yaw_rate", rr_Scalar(float(gyro[2])))
                        rr.log("imu/accel/forward_x", rr_Scalar(float(accel[0])))
                    except Exception as e:
                        print(f"[Visualizer] BNO log error: {e}")

                # 3. Handle Controller Commands
                elif input_id == "control_cmd":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        cmd = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        rr.log("control/throttle", rr_Scalar(float(cmd.get("throttle", cmd.get("vx", 0.0)))))
                        rr.log("control/steer_yaw", rr_Scalar(float(cmd.get("steer_yaw", cmd.get("vyaw", 0.0)))))
                        rr.log("control/steer_pitch", rr_Scalar(float(cmd.get("steer_pitch", cmd.get("pitch", 0.0)))))
                        rr.log("control/stick_right_x", rr_Scalar(float(cmd.get("stick_right_x", cmd.get("steer_yaw", 0.0)))))
                        rr.log("control/stick_right_y", rr_Scalar(float(cmd.get("stick_right_y", cmd.get("steer_pitch", 0.0)))))
                        rr.log("control/ballast_cmd", rr_Scalar(float(cmd.get("ballast", 0.0))))
                        rr.log("control/brake", rr_Scalar(1.0 if cmd.get("brake", False) else 0.0))
                        rr.log("control/e_stop", rr_Scalar(1.0 if cmd.get("e_stop", False) else 0.0))
                    except Exception as e:
                        print(f"[Visualizer] Controller log error: {e}")

                # 4. Handle ESP Status (TCP Health & Latency)
                elif input_id == "esp_status":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        status = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        esp1_conn = 1.0 if status.get("esp1", {}).get("connected", False) else 0.0
                        esp2_conn = 1.0 if status.get("esp2", {}).get("connected", False) else 0.0
                        rr.log("tcp_health/esp1_connected", rr_Scalar(esp1_conn))
                        rr.log("tcp_health/esp2_connected", rr_Scalar(esp2_conn))
                        rr.log("tcp_health/esp1_latency_ms", rr_Scalar(float(status.get("esp1", {}).get("latency_ms", 0.0))))
                        rr.log("tcp_health/esp2_latency_ms", rr_Scalar(float(status.get("esp2", {}).get("latency_ms", 0.0))))
                        rr.log("status_text/esp1", rr.TextLog(f"ESP1 (Thrust/Steer): {status.get('esp1', {}).get('state', 'UNKNOWN')}"))
                        rr.log("status_text/esp2", rr.TextLog(f"ESP2 (Ballast): {status.get('esp2', {}).get('state', 'UNKNOWN')}"))
                    except Exception as e:
                        print(f"[Visualizer] ESP status log error: {e}")

                # 5. Handle ESP Telemetry (Sensors, Voltage, Actual Actuator feedback)
                elif input_id == "esp_telemetry":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        telemetry = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        if "esp1" in telemetry:
                            t1 = telemetry["esp1"]
                            rr.log("power/esp1_voltage", rr_Scalar(float(t1.get("voltage", 12.0))))
                            rr.log("power/esp1_current", rr_Scalar(float(t1.get("current", 0.5))))
                        if "esp2" in telemetry:
                            t2 = telemetry["esp2"]
                            rr.log("power/esp2_voltage", rr_Scalar(float(t2.get("voltage", 12.0))))
                            rr.log("power/esp2_current", rr_Scalar(float(t2.get("current", 0.5))))
                            if "water_depth_m" in t2:
                                rr.log("sensor/water_depth_m", rr_Scalar(float(t2["water_depth_m"])))
                    except Exception as e:
                        print(f"[Visualizer] ESP telemetry log error: {e}")

                # 6. Handle Compute Status (Actuator Allocations & State Machine)
                elif input_id == "compute_status":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        c_status = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        rr.log("status_text/robot_state", rr.TextLog(f"AUV State: {c_status.get('robot_state', 'UNKNOWN')}"))
                        rr.log("actuators/bldc_thrust_pwm", rr_Scalar(float(c_status.get("throttle_pct", 0))))

                        # Log 4 membrane leg deployment angles (degrees)
                        leg_angles = c_status.get("leg_deploy_angles", [0.0, 0.0, 0.0, 0.0])
                        if len(leg_angles) >= 4:
                            rr.log("actuators/legs/0_top_deploy_deg", rr_Scalar(float(leg_angles[0])))
                            rr.log("actuators/legs/1_right_deploy_deg", rr_Scalar(float(leg_angles[1])))
                            rr.log("actuators/legs/2_bottom_deploy_deg", rr_Scalar(float(leg_angles[2])))
                            rr.log("actuators/legs/3_left_deploy_deg", rr_Scalar(float(leg_angles[3])))

                        # Log head ballast intake
                        rr.log("actuators/ballast/fill_ratio", rr_Scalar(float(c_status.get("ballast_fill_ratio", 0.5))))
                        rr.log("actuators/ballast/servo_deg", rr_Scalar(float(c_status.get("ballast_intake_deg", 0.0))))
                        rr.log("compute/dt_ms", rr_Scalar(float(c_status.get("dt_ms", 0.0))))
                    except Exception as e:
                        print(f"[Visualizer] Compute status log error: {e}")

                # 7. Handle Target Balloon Relative Telemetry
                elif input_id == "target_relative_info":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        target_info = json.loads(raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8"))

                        if target_info.get("target_found", False):
                            dist = float(target_info.get("distance_m", 0.0))
                            az = float(target_info.get("azimuth_deg", 0.0))
                            el = float(target_info.get("elevation_deg", 0.0))
                            pop_cnt = int(target_info.get("pop_count", 0))

                            rr.log("balloon/distance_m", rr_Scalar(dist))
                            rr.log("balloon/azimuth_deg", rr_Scalar(az))
                            rr.log("balloon/elevation_deg", rr_Scalar(el))
                            rr.log("balloon/popped_count", rr_Scalar(pop_cnt))

                            pos_w = target_info.get("target_pos_world", [0, 0, 0])
                            rr.log("world/balloon_target", rr.Points3D([pos_w], radii=0.20, colors=[[255, 30, 80]]))

                            if target_info.get("just_popped", False):
                                rr.log("status_text/balloon", rr.TextLog(f"*** BALLOON DESTROYED! Count: {pop_cnt} ***"))
                    except Exception as e:
                        print(f"[Visualizer] Target info log error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        os._exit(0)


if __name__ == "__main__":
    main()
