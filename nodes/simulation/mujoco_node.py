"""
MuJoCo Physics Simulation Node for QuadKen Underwater AUV.

Integrates with dora-rs dataflow:
  Inputs:
    - tick: Simulation step timer (e.g. 20ms / 50Hz)
    - actuator_cmd: Actuator setpoints from nodes/pc/compute.py
        * esp1.motors: Dual BLDC forward thruster PWMs (0-100)
        * esp1.servos: 4x Membrane leg deployment angles [Top, Right, Bottom, Left] (0-70 deg)
        * esp2.servos: 4x Head water-intake ballast servos (0-90 deg)
  Outputs:
    - bno_data: Virtual BNO055 orientation, gyro, accel, and depth (for compute & visualizer)
    - image: Virtual underwater forward camera feed with HUD (for visualizer)
    - image_overhead: Virtual third-person chase/overhead camera feed with HUD (for visualizer)
    - esp_status: Simulated ESP1 & ESP2 TCP connection health (for compute & visualizer)
    - esp_telemetry: Simulated ESP telemetry like battery & depth (for compute & visualizer)
"""

import os
import sys
import time
import json
import math
import cv2
import numpy as np
import pyarrow as pa
import mujoco
from dora import Node

MODEL_PATH = "assets/quadken.xml"


def quat_to_euler_deg(w, x, y, z):
    """Convert quaternion [w, x, y, z] to Euler angles in degrees (roll, pitch, yaw)."""
    # Roll (X-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class QuadKenMuJoCoSim:
    def __init__(self, xml_path=MODEL_PATH):
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MuJoCo XML model not found at: {xml_path}")

        print(f"[MuJoCo Node] Loading model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Offscreen camera renderer (Width 320, Height 240)
        self.cam_width = 320
        self.cam_height = 240
        self.renderer = mujoco.Renderer(self.model, height=self.cam_height, width=self.cam_width)

        # Actuator parameters
        self.max_thrust_n = 200.0  # Max thrust per BLDC thruster (N)

        # Cached actuator commands
        self.target_bldc = [0.0, 0.0]        # PWM 0-100
        self.target_legs = [0.0, 0.0, 0.0, 0.0]  # [Top, Right, Bottom, Left] in degrees
        self.target_ballast = [0.0, 0.0, 0.0, 0.0]

        # Hydrodynamics parameters
        self.water_density = 1000.0  # kg/m^3
        self.membrane_area_max = 0.035  # m^2 per membrane segment
        self.drag_coeff = 1.2
        self.lever_arm = 0.16  # distance from body centerline to membrane center (m)

        # Timing
        self.last_render_time = 0.0
        self.render_interval = 0.04  # ~25 FPS for camera rendering
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    def set_actuator_commands(self, cmd_dict):
        """Parse actuator_cmd from pc/compute.py."""
        esp1 = cmd_dict.get("esp1", {})
        esp2 = cmd_dict.get("esp2", {})

        motors = esp1.get("motors", [])
        if len(motors) >= 2:
            self.target_bldc = [float(motors[0]), float(motors[1])]

        servos1 = esp1.get("servos", [])
        if len(servos1) >= 4:
            self.target_legs = [float(servos1[i]) for i in range(4)]

        servos2 = esp2.get("servos", [])
        if len(servos2) >= 4:
            self.target_ballast = [float(servos2[i]) for i in range(4)]

    def apply_control_and_hydrodynamics(self):
        """Apply actuator targets and hydrodynamic drag steering moments."""
        # 1. Apply BLDC forward thrust (Actuators 0 & 1)
        thrust_1 = (max(0.0, min(100.0, self.target_bldc[0])) / 100.0) * self.max_thrust_n
        thrust_2 = (max(0.0, min(100.0, self.target_bldc[1])) / 100.0) * self.max_thrust_n
        self.data.ctrl[0] = thrust_1
        self.data.ctrl[1] = thrust_2

        # 2. Apply Leg deployment servos (Actuators 2..5, degrees [0..90])
        for i in range(4):
            self.data.ctrl[2 + i] = max(0.0, min(90.0, self.target_legs[i]))

        # 3. Apply Ballast servos (Actuators 6..9, degrees [0..90])
        for i in range(4):
            self.data.ctrl[6 + i] = max(0.0, min(90.0, self.target_ballast[i]))

        # 4. Hydrodynamic Drag Steering Calculation
        # Compute body-frame forward velocity u
        rot_mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_mat, self.data.qpos[3:7])
        rot_mat = rot_mat.reshape((3, 3))

        world_lin_vel = self.data.qvel[0:3]
        body_lin_vel = rot_mat.T @ world_lin_vel
        u_forward = max(0.0, body_lin_vel[0])  # Robot forward speed (+X)

        # Membrane drag forces when legs are open
        # Legs: [0: Top, 1: Right, 2: Bottom, 3: Left]
        top_deg = self.target_legs[0]
        right_deg = self.target_legs[1]
        bottom_deg = self.target_legs[2]
        left_deg = self.target_legs[3]

        def calc_drag(angle_deg):
            area = self.membrane_area_max * math.sin(math.radians(max(0.0, min(90.0, angle_deg))))
            return 0.5 * self.water_density * (u_forward ** 2) * self.drag_coeff * area

        d_top = calc_drag(top_deg)
        d_right = calc_drag(right_deg)
        d_bottom = calc_drag(bottom_deg)
        d_left = calc_drag(left_deg)

        # Hydrodynamic Moments in body frame:
        # Yaw torque: Right leg drag creates +Z (turn right), Left creates -Z (turn left)
        tau_yaw = (d_right - d_left) * self.lever_arm
        # Pitch torque: Top leg drag creates +Y (pitch down / dive), Bottom creates -Y (pitch up / ascend)
        tau_pitch = (d_top - d_bottom) * self.lever_arm
        # Total braking drag along -X
        total_drag = d_top + d_right + d_bottom + d_left

        body_force = np.array([-total_drag, 0.0, 0.0])
        body_torque = np.array([0.0, tau_pitch, tau_yaw])

        # Transform to world coordinates and apply to auv root body
        world_force = rot_mat @ body_force
        world_torque = rot_mat @ body_torque

        # Apply external force & torque to root body (body id 1: auv)
        self.data.xfrc_applied[1, 0:3] = world_force
        self.data.xfrc_applied[1, 3:6] = world_torque

    def step(self):
        """Advance MuJoCo physics by one step."""
        self.apply_control_and_hydrodynamics()
        mujoco.mj_step(self.model, self.data)

    def get_bno_payload(self, seq):
        """Extract virtual BNO055 telemetry."""
        quat = self.data.sensor("imu_quat").data  # [w, x, y, z]
        gyro = self.data.sensor("imu_gyro").data  # [gx, gy, gz] in rad/s
        accel = self.data.sensor("imu_accel").data  # [ax, ay, az] in m/s^2

        roll, pitch, yaw = quat_to_euler_deg(quat[0], quat[1], quat[2], quat[3])

        # Depth: -Z position in world frame (Z=-1.5m -> Depth 1.5m)
        depth_m = max(0.0, -float(self.data.qpos[2]))

        gyro_deg = [float(np.rad2deg(g)) for g in gyro]
        accel_data = [float(a) for a in accel]

        payload = {
            "seq": seq,
            "timestamp": time.time(),
            "roll": round(float(roll), 2),
            "pitch": round(float(pitch), 2),
            "yaw": round(float(yaw), 2),
            "gyro": [round(g, 2) for g in gyro_deg],
            "accel": [round(a, 2) for a in accel_data],
            "depth_m": round(float(depth_m), 2),
        }
        return payload

    def render_front_camera(self, bno_data):
        """Render front camera image and add HUD overlay."""
        self.renderer.update_scene(self.data, camera="front_camera")
        rgb_img = self.renderer.render()
        bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

        # Draw HUD overlays on camera frame
        h, w = bgr_img.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Target HUD Reticle (crosshair)
        cv2.line(bgr_img, (cx - 14, cy), (cx + 14, cy), (0, 240, 240), 1)
        cv2.line(bgr_img, (cx, cy - 14), (cx, cy + 14), (0, 240, 240), 1)
        cv2.circle(bgr_img, (cx, cy), 8, (0, 240, 240), 1)

        # 2. Header HUD: Mode & Depth
        cv2.putText(
            bgr_img,
            "MUJOCO VIRTUAL CAM",
            (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        depth = bno_data.get("depth_m", 1.5)
        pitch = bno_data.get("pitch", 0.0)
        yaw = bno_data.get("yaw", 0.0)
        hud_str = f"DEPTH:{depth:4.1f}m  PITCH:{pitch:+4.1f}deg  YAW:{yaw:+4.1f}deg"
        cv2.putText(
            bgr_img,
            hud_str,
            (8, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (200, 255, 200),
            1,
            cv2.LINE_AA,
        )

        # Compress to JPEG bytes
        success, encoded_jpg = cv2.imencode(".jpg", bgr_img, self.encode_param)
        if success:
            return encoded_jpg.tobytes()
        return None

    def render_overhead_camera(self, bno_data):
        """Render third-person overhead chase camera image and add HUD overlay."""
        self.renderer.update_scene(self.data, camera="overhead_camera")
        rgb_img = self.renderer.render()
        bgr_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

        h, w = bgr_img.shape[:2]

        # Header HUD
        cv2.putText(
            bgr_img,
            "MUJOCO CHASE / OVERHEAD CAM",
            (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Footer HUD: Forward Thrust and 4-Leg deploy angles
        thrust_pct = int(self.target_bldc[0])
        legs_str = f"THRUST:{thrust_pct}%  LEGS:[T:{int(self.target_legs[0])} R:{int(self.target_legs[1])} B:{int(self.target_legs[2])} L:{int(self.target_legs[3])}]"
        cv2.putText(
            bgr_img,
            legs_str,
            (8, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (255, 200, 100),
            1,
            cv2.LINE_AA,
        )

        # Compress to JPEG bytes
        success, encoded_jpg = cv2.imencode(".jpg", bgr_img, self.encode_param)
        if success:
            return encoded_jpg.tobytes()
        return None


def main():
    node = Node()
    sim = QuadKenMuJoCoSim()

    seq = 0
    last_camera_time = 0.0
    camera_period = 0.04  # 25 FPS

    print("[MuJoCo Sim Node] Initialized and running physics simulation.")

    try:
        for event in node:
            event_type = event["type"]

            if event_type == "STOP":
                print("[MuJoCo Sim Node] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                input_id = event["id"]
                raw_value = event["value"]

                if input_id == "actuator_cmd":
                    try:
                        raw_bytes = raw_value.to_pylist()[0]
                        cmd_dict = json.loads(
                            raw_bytes if isinstance(raw_bytes, str) else raw_bytes.decode("utf-8")
                        )
                        sim.set_actuator_commands(cmd_dict)
                    except Exception as e:
                        print(f"[MuJoCo Sim Node] Actuator cmd decode error: {e}")

                elif input_id == "tick":
                    seq += 1
                    now = time.time()

                    # Step physics
                    sim.step()

                    # 1. Publish Virtual BNO055 IMU data (50 Hz)
                    bno_payload = sim.get_bno_payload(seq)
                    node.send_output(
                        "bno_data",
                        pa.array([json.dumps(bno_payload).encode("utf-8")]),
                    )

                    # 2. Publish Simulated ESP Status & Telemetry
                    esp_status = {
                        "esp1": {"connected": True, "latency_ms": 0.8, "state": "READY (MuJoCo)"},
                        "esp2": {"connected": True, "latency_ms": 0.8, "state": "READY (MuJoCo)"},
                    }
                    node.send_output(
                        "esp_status",
                        pa.array([json.dumps(esp_status).encode("utf-8")]),
                    )

                    esp_telemetry = {
                        "esp1": {
                            "battery_v": 12.4,
                            "depth_m": bno_payload["depth_m"],
                            "servos": sim.target_legs,
                            "motors": sim.target_bldc,
                        },
                        "esp2": {
                            "battery_v": 12.3,
                            "servos": sim.target_ballast,
                        },
                    }
                    node.send_output(
                        "esp_telemetry",
                        pa.array([json.dumps(esp_telemetry).encode("utf-8")]),
                    )

                    # 3. Publish Virtual Camera Images (25 FPS)
                    if now - last_camera_time >= camera_period:
                        last_camera_time = now
                        # Front camera feed
                        jpeg_bytes = sim.render_front_camera(bno_payload)
                        if jpeg_bytes is not None:
                            node.send_output("image", pa.array([jpeg_bytes]))
                        # Third-person overhead chase camera feed
                        jpeg_bytes_overhead = sim.render_overhead_camera(bno_payload)
                        if jpeg_bytes_overhead is not None:
                            node.send_output("image_overhead", pa.array([jpeg_bytes_overhead]))

    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
