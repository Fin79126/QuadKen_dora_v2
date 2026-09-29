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

try:
    from config.robot_config import LEG_SERVO_MAX_SPEED_DPS
except ImportError:
    LEG_SERVO_MAX_SPEED_DPS = 500.0

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
        mujoco.mj_forward(self.model, self.data)

        # Offscreen camera renderer (Width 320, Height 240)
        self.cam_width = 320
        self.cam_height = 240
        self.renderer = mujoco.Renderer(self.model, height=self.cam_height, width=self.cam_width)

        # Actuator parameters
        self.max_thrust_n = 300.0  # Max thrust per BLDC thruster (N)

        # Cached actuator commands
        self.target_bldc = [0.0, 0.0]        # PWM 0-100
        self.target_legs = [0.0, 0.0, 0.0, 0.0]  # [Top, Right, Bottom, Left] in degrees
        self.target_ballast = [0.0, 0.0, 0.0, 0.0]

        # Resolve AUV root body ID dynamically from XML model
        self.auv_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "auv")
        if self.auv_body_id == -1:
            raise ValueError("Body 'auv' not found in MuJoCo model!")

        # Hydrodynamics parameters
        self.water_density = 1000.0   # kg/m^3
        self.membrane_area_max = 0.08 # m^2 per deployed membrane quadrant (kept as requested)
        self.drag_coeff = 0.6         # Longitudinal drag coefficient (Cd, moderate flexible membrane)
        self.lift_coeff = 0.35        # Normal / lateral steering force coefficient (Cl, moderate flexible membrane)
        self.x_com = 0.26             # Center of Mass X in body frame (m, from quadken.xml)
        self.z_com = 0.0              # Center of Mass Z in body frame (aligned on centerline)
        self.leg_length = 0.30        # Leg strut length (m)
        self.hull_radius = 0.12       # Hull radius at aft hinge rim (m)

        # Servo physical rate limiter (realistic high-speed underwater servo: 500 deg/s)
        self.current_leg_ctrl = [0.0, 0.0, 0.0, 0.0]
        self.max_leg_servo_speed = LEG_SERVO_MAX_SPEED_DPS  # deg/s (~0.12s per 60 deg)

        # Joint indices for physical leg hinge joints
        self.leg_joint_names = ["joint_leg_top", "joint_leg_right", "joint_leg_bottom", "joint_leg_left"]
        self.leg_qpos_indices = [
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in self.leg_joint_names
        ]

        # Timing
        self.last_render_time = 0.0
        self.render_interval = 0.04  # ~25 FPS for camera rendering
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

        # Free-chase camera that tracks AUV position & yaw without rolling (keeps horizon level)
        self.chase_cam = mujoco.MjvCamera()
        self.chase_cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.chase_cam.distance = 1.6
        self.chase_cam.elevation = -26.0

        # Target Balloon Setup (Mocap Body for collision & dynamic respawning)
        self.balloon_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "balloon")
        self.balloon_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "balloon_geom")
        if self.balloon_body_id != -1 and self.model.body_mocapid[self.balloon_body_id] != -1:
            self.balloon_mocap_id = int(self.model.body_mocapid[self.balloon_body_id])
        else:
            self.balloon_mocap_id = 0 if self.model.nmocap > 0 else -1

        self.balloon_pop_count = 0
        self.last_pop_time = 0.0
        self.is_surfaced = False

        # Submerged target waypoints distributed across the 36m x 21m pool arena (X: 5..25m, Y: -4..+4m, Z: -1.4..-2.5m)
        self.balloon_waypoints = [
            [5.0, 0.5, -1.8],
            [12.0, -3.0, -2.2],
            [18.0, 4.0, -1.6],
            [25.0, -2.0, -2.5],
            [14.0, 2.5, -1.4],
            [7.0, -3.5, -2.0],
            [22.0, 0.0, -1.8],
        ]
        self.current_waypoint_idx = 0

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
        """Apply actuator targets, hydrodynamic drag steering, and head ballast buoyancy shifts."""
        # Water Surface Boundary Dynamics (Surface at Z = 0.0m)
        # Main hull radius is 0.12m. Center of robot is self.data.qpos[2].
        # Hull top is at z_robot + 0.12m.
        z_robot = float(self.data.qpos[2])
        # submerged_ratio: 1.0 when fully submerged (z <= -0.12m), 0.0 when fully in air (z >= +0.12m)
        submerged_ratio = max(0.0, min(1.0, (-z_robot + 0.12) / 0.24))
        self.is_surfaced = (z_robot >= -0.10)

        # 1. Apply BLDC forward thrust (Actuators 0 & 1), scaled by water immersion
        thrust_1 = (max(0.0, min(100.0, self.target_bldc[0])) / 100.0) * self.max_thrust_n * submerged_ratio
        thrust_2 = (max(0.0, min(100.0, self.target_bldc[1])) / 100.0) * self.max_thrust_n * submerged_ratio
        self.data.ctrl[0] = thrust_1
        self.data.ctrl[1] = thrust_2

        # 2. Apply Leg deployment servos with physical rate limiting (Actuators 2..5, degrees [0..90])
        dt = float(self.model.opt.timestep)
        max_delta = self.max_leg_servo_speed * dt
        for i in range(4):
            target = max(0.0, min(90.0, self.target_legs[i]))
            delta = target - self.current_leg_ctrl[i]
            if abs(delta) > max_delta:
                self.current_leg_ctrl[i] += math.copysign(max_delta, delta)
            else:
                self.current_leg_ctrl[i] = target
            self.data.ctrl[2 + i] = self.current_leg_ctrl[i]

        # 3. Apply Ballast servos (Actuators 6..9, degrees [0..90])
        for i in range(4):
            self.data.ctrl[6 + i] = max(0.0, min(90.0, self.target_ballast[i]))

        # 4. Hydrodynamic Drag Steering & Turning Moments Calculation
        rot_mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_mat, self.data.qpos[3:7])
        rot_mat = rot_mat.reshape((3, 3))

        world_lin_vel = self.data.qvel[0:3]
        body_lin_vel = rot_mat.T @ world_lin_vel
        u_forward = max(0.0, body_lin_vel[0])  # Robot forward speed (+X)

        total_body_force = np.zeros(3, dtype=np.float64)
        total_body_torque = np.zeros(3, dtype=np.float64)

        if u_forward > 0.01:
            q_dynamic = 0.5 * self.water_density * (u_forward ** 2) * submerged_ratio

            # Legs: [0: Top, 1: Right, 2: Bottom, 3: Left]
            # Uses ACTUAL physical joint angle (qpos) instead of target command for realistic transient response!
            for i in range(4):
                qpos_idx = self.leg_qpos_indices[i]
                theta_rad = float(self.data.qpos[qpos_idx])
                theta_rad = max(0.0, min(math.pi / 2.0, theta_rad))
                angle_deg = math.degrees(theta_rad)
                if angle_deg < 0.5:
                    continue

                sin_th = math.sin(theta_rad)
                cos_th = math.cos(theta_rad)

                # Projected area, longitudinal drag, and normal (rudder/lift) force
                area = self.membrane_area_max * sin_th
                f_drag = q_dynamic * self.drag_coeff * area
                f_normal = q_dynamic * self.lift_coeff * area * cos_th

                # Center of pressure of deployed membrane relative to CoM
                x_cp = -0.5 * self.leg_length * cos_th
                r_cp = self.hull_radius + 0.5 * self.leg_length * sin_th
                delta_x = x_cp - self.x_com

                # Leg 0: Top (+Z hinge, opens upward)
                # Leg 1: Right (-Y hinge, opens rightward)
                # Leg 2: Bottom (-Z hinge, opens downward)
                # Leg 3: Left (+Y hinge, opens leftward)
                if i == 0:  # Top
                    r_vec = np.array([delta_x, 0.0, r_cp - self.z_com])
                    f_vec = np.array([-f_drag, 0.0, -f_normal])  # Inward normal force (-Z)
                elif i == 1:  # Right
                    r_vec = np.array([delta_x, -r_cp, 0.0 - self.z_com])
                    f_vec = np.array([-f_drag, +f_normal, 0.0])  # Inward normal force (+Y)
                elif i == 2:  # Bottom
                    r_vec = np.array([delta_x, 0.0, -r_cp - self.z_com])
                    f_vec = np.array([-f_drag, 0.0, +f_normal])  # Inward normal force (+Z)
                elif i == 3:  # Left
                    r_vec = np.array([delta_x, +r_cp, 0.0 - self.z_com])
                    f_vec = np.array([-f_drag, -f_normal, 0.0])  # Inward normal force (-Y)

                torque_vec = np.cross(r_vec, f_vec)
                total_body_force += f_vec
                total_body_torque += torque_vec

        # Transform hydrodynamics to world coordinates
        world_force = rot_mat @ total_body_force
        world_torque = rot_mat @ total_body_torque

        # 5. Underwater Archimedes Buoyancy & Ballast Trim Dynamics:
        # AUV subtree weight is ~74.16 N (7.56 kg * 9.81 m/s^2).
        # Empty (ratio=0): Net positive buoyancy (+2.0 N upward) with CoB forward at X=0.234m,
        # producing a natural nose-up pitch trim (+7 deg) and gentle surfacing.
        # Ballasted (ratio>0): Ingests water at head ballast tank (X=0.65m), adding up to 3.5 N
        # downward weight, which shifts CoM ahead of CoB (longitudinal trim reversal) and produces
        # negative buoyancy (-1.5 N net downward), diving nose-first.
        com_world = self.data.xipos[self.auv_body_id]

        # Base upward buoyancy scaled by submerged fraction (at Z=0, buoyancy drops and gravity pulls AUV back down!)
        f_buoy_mag = 76.16 * submerged_ratio
        f_buoy_world = np.array([0.0, 0.0, f_buoy_mag])
        p_cob_world = self.data.qpos[0:3] + rot_mat @ np.array([0.234, 0.0, 0.0])
        r_cob = p_cob_world - com_world
        tau_buoy = np.cross(r_cob, f_buoy_world)

        world_force += f_buoy_world
        world_torque += tau_buoy

        # Head Ballast Water Intake (ESP2)
        ballast_ratio = (np.mean(self.target_ballast) / 90.0) if self.target_ballast else 0.0
        ballast_ratio = max(0.0, min(1.0, ballast_ratio))

        if ballast_ratio > 0.001:
            f_ballast_down = 3.5 * ballast_ratio
            f_ballast_world = np.array([0.0, 0.0, -f_ballast_down])
            p_ballast_world = self.data.qpos[0:3] + rot_mat @ np.array([0.65, 0.0, 0.0])
            r_ballast = p_ballast_world - com_world
            tau_ballast = np.cross(r_ballast, f_ballast_world)

            world_force += f_ballast_world
            world_torque += tau_ballast

        # 6. Water Fluid Viscous Damping (Linear drag & selective angular damping)
        f_viscous_damping = -15.0 * world_lin_vel
        # Enhanced angular damping in water (pitch & yaw rotational drag + quadratic damping)
        w_world = self.data.qvel[3:6]
        w_body = rot_mat.T @ w_world
        tau_damp_pitch = -7.0 * w_body[1] - 3.0 * w_body[1] * abs(w_body[1])
        tau_damp_yaw   = -7.0 * w_body[2] - 3.0 * w_body[2] * abs(w_body[2])
        tau_damp_roll  = -0.4 * w_body[0]
        tau_damp_body = np.array([tau_damp_roll, tau_damp_pitch, tau_damp_yaw])
        tau_viscous_damping = rot_mat @ tau_damp_body

        world_force += f_viscous_damping
        world_torque += tau_viscous_damping

        # Apply total external forces & torques to auv root body
        self.data.xfrc_applied[self.auv_body_id, 0:3] = world_force
        self.data.xfrc_applied[self.auv_body_id, 3:6] = world_torque

    def check_balloon_collision(self):
        """Check for collision or close proximity between QuadKen nose and balloon."""
        if self.balloon_mocap_id == -1:
            return

        now = time.time()
        if now - self.last_pop_time < 1.0:
            return

        rot_mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_mat, self.data.qpos[3:7])
        rot_mat = rot_mat.reshape((3, 3))
        p_robot = self.data.qpos[0:3]
        p_nose = p_robot + rot_mat @ np.array([0.72, 0.0, 0.0])

        balloon_pos = self.data.mocap_pos[self.balloon_mocap_id]
        dist_to_nose = float(np.linalg.norm(balloon_pos - p_nose))

        # Check contact list in MuJoCo
        contact_detected = False
        if self.balloon_geom_id != -1:
            for c_idx in range(self.data.ncon):
                contact = self.data.contact[c_idx]
                if contact.geom1 == self.balloon_geom_id or contact.geom2 == self.balloon_geom_id:
                    contact_detected = True
                    break

        # Strike threshold: nose collision or within 0.35m of balloon center
        if contact_detected or dist_to_nose < 0.35:
            self.balloon_pop_count += 1
            self.last_pop_time = now
            print(f"\n[MuJoCo Node] *******************************************")
            print(f"[MuJoCo Node] *** BALLOON POPPED! Total Count: {self.balloon_pop_count} ***")
            print(f"[MuJoCo Node] *******************************************\n")

            # Advance to next waypoint
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.balloon_waypoints)
            next_pos = self.balloon_waypoints[self.current_waypoint_idx]
            self.data.mocap_pos[self.balloon_mocap_id] = next_pos
            mujoco.mj_forward(self.model, self.data)

    def step(self):
        """Advance MuJoCo physics by one step."""
        self.apply_control_and_hydrodynamics()
        mujoco.mj_step(self.model, self.data)
        self.check_balloon_collision()

    def get_target_relative_info(self):
        """Compute relative azimuth, elevation, and distance to balloon from AUV nose."""
        if self.balloon_mocap_id == -1:
            return {
                "target_found": False,
                "azimuth_deg": 0.0,
                "elevation_deg": 0.0,
                "distance_m": 0.0,
                "target_pos_world": [0.0, 0.0, 0.0],
                "pop_count": self.balloon_pop_count,
                "just_popped": False,
            }

        rot_mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(rot_mat, self.data.qpos[3:7])
        rot_mat = rot_mat.reshape((3, 3))
        p_robot = self.data.qpos[0:3]
        p_nose = p_robot + rot_mat @ np.array([0.72, 0.0, 0.0])

        balloon_pos = self.data.mocap_pos[self.balloon_mocap_id]
        v_world = balloon_pos - p_nose
        v_body = rot_mat.T @ v_world

        xb, yb, zb = v_body
        dist = float(np.linalg.norm(v_body))
        # Robot convention: Forward is +X, Left is +Y, Up is +Z
        # Azimuth: Left is negative, Right is positive (-yb)
        azimuth_deg = math.degrees(math.atan2(-yb, max(1e-4, xb))) if xb > 0 else math.degrees(math.atan2(-yb, xb))
        # Elevation: Down is negative, Up is positive (+zb)
        elevation_deg = math.degrees(math.atan2(zb, math.hypot(xb, yb)))

        just_popped = (time.time() - self.last_pop_time < 1.5)

        return {
            "target_found": True,
            "azimuth_deg": round(float(azimuth_deg), 2),
            "elevation_deg": round(float(elevation_deg), 2),
            "distance_m": round(float(dist), 2),
            "target_pos_world": [round(float(p), 2) for p in balloon_pos],
            "pop_count": self.balloon_pop_count,
            "just_popped": just_popped,
            "is_surfaced": bool(getattr(self, "is_surfaced", False)),
        }

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
            "is_surfaced": bool(getattr(self, "is_surfaced", False)),
        }
        return payload

    def render_front_camera(self, bno_data, target_info=None):
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

        # 2. Header HUD: Mode & Target Info
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

        if target_info and target_info.get("target_found", False):
            dist = target_info.get("distance_m", 0.0)
            az = target_info.get("azimuth_deg", 0.0)
            el = target_info.get("elevation_deg", 0.0)
            pop = target_info.get("pop_count", 0)
            hud_target = f"BALLOON:{dist:4.1f}m [Az:{az:+4.1f} El:{el:+4.1f}] POP:{pop}"
            cv2.putText(
                bgr_img,
                hud_target,
                (8, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.33,
                (100, 220, 255),
                1,
                cv2.LINE_AA,
            )

            if target_info.get("just_popped"):
                cv2.rectangle(bgr_img, (cx - 90, cy - 16), (cx + 90, cy + 16), (0, 200, 50), -1)
                cv2.putText(
                    bgr_img,
                    "TARGET DESTROYED!",
                    (cx - 80, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
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

        # Surface breach warning overlay
        if bno_data.get("is_surfaced", False):
            cv2.rectangle(bgr_img, (cx - 95, 40), (cx + 95, 60), (0, 0, 180), -1)
            cv2.putText(
                bgr_img,
                "SURFACE BREACH",
                (cx - 75, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        # Compress to JPEG bytes
        success, encoded_jpg = cv2.imencode(".jpg", bgr_img, self.encode_param)
        if success:
            return encoded_jpg.tobytes()
        return None

    def render_overhead_camera(self, bno_data, target_info=None):
        """Render third-person chase camera with level horizon (roll-free view)."""
        yaw_deg = float(bno_data.get("yaw", 0.0))
        # Look at the center of the AUV body
        self.chase_cam.lookat = self.data.xpos[self.auv_body_id].copy()
        # Azimuth follows yaw so the camera is positioned behind the AUV, looking forward.
        # In MuJoCo free camera, azimuth=0 means the camera is at -X looking towards +X (forward).
        self.chase_cam.distance = 1.7
        self.chase_cam.azimuth = yaw_deg
        self.chase_cam.elevation = -26.0

        self.renderer.update_scene(self.data, camera=self.chase_cam)
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

        if bno_data.get("is_surfaced", False):
            cv2.putText(
                bgr_img,
                "[SURFACED]",
                (w - 95, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 90, 255),
                1,
                cv2.LINE_AA,
            )

        if target_info and target_info.get("target_found", False):
            dist = target_info.get("distance_m", 0.0)
            pop = target_info.get("pop_count", 0)
            cv2.putText(
                bgr_img,
                f"TARGET BALLOON: {dist:4.1f}m | DESTROYED: {pop}",
                (8, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.33,
                (100, 220, 255),
                1,
                cv2.LINE_AA,
            )
            if target_info.get("just_popped"):
                cv2.putText(
                    bgr_img,
                    "*** TARGET DESTROYED! ***",
                    (w // 2 - 80, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 100),
                    1,
                    cv2.LINE_AA,
                )

        # Footer HUD: Forward Thrust, Ballast intake %, and 4-Leg deploy angles
        thrust_pct = int(self.target_bldc[0])
        ballast_pct = int((np.mean(self.target_ballast) / 90.0) * 100) if self.target_ballast else 0
        legs_str = f"THRUST:{thrust_pct}%  BALLAST:{ballast_pct}%  LEGS:[T:{int(self.target_legs[0])} R:{int(self.target_legs[1])} B:{int(self.target_legs[2])} L:{int(self.target_legs[3])}]"
        cv2.putText(
            bgr_img,
            legs_str,
            (8, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.30,
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

                    # 3. Publish Target Balloon Relative Info (Ground Truth for Step 2)
                    target_info = sim.get_target_relative_info()
                    node.send_output(
                        "target_relative_info",
                        pa.array([json.dumps(target_info).encode("utf-8")]),
                    )

                    # 4. Publish Virtual Camera Images (25 FPS)
                    if now - last_camera_time >= camera_period:
                        last_camera_time = now
                        # Front camera feed
                        jpeg_bytes = sim.render_front_camera(bno_payload, target_info)
                        if jpeg_bytes is not None:
                            node.send_output("image", pa.array([jpeg_bytes]))
                        # Third-person overhead chase camera feed
                        jpeg_bytes_overhead = sim.render_overhead_camera(bno_payload, target_info)
                        if jpeg_bytes_overhead is not None:
                            node.send_output("image_overhead", pa.array([jpeg_bytes_overhead]))

    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
