"""
Autonomous AI Guidance Node for QuadKen Underwater AUV (Step 2 GNC).

Subscribes to:
  - target_relative_info: Relative azimuth, elevation, and distance to balloon (Ground Truth or Perception)
  - bno_data: Virtual or physical BNO055 orientation, angular rates, depth, and surface breach telemetry
  - tick: Periodic control clock (e.g. 20ms / 50Hz)

Publishes:
  - control_cmd: Throttle, yaw/pitch drag steering, stick vectors, ballast level, and mission mode
  - guidance_status: High-level GNC state, tracking errors, and telemetry for Rerun visualization
"""

import os
import sys
import time
import json
import math
import numpy as np
import pyarrow as pa
from dora import Node

# Ensure stdout and stderr use UTF-8 on Windows to avoid dora daemon UTF-8 warnings
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Optional OpenTelemetry tracing
try:
    from opentelemetry import trace
    tracer = trace.get_tracer("quadken.pc.ai_guidance")
except ImportError:
    tracer = None


class AIGuidanceController:
    """
    Guidance, Navigation, and Control (GNC) algorithm for QuadKen AUV.
    Translates target relative geometry (azimuth, elevation, distance) and
    BNO IMU dynamics into drag-steering, ballast trim, and BLDC thrust commands.
    """

    def __init__(self):
        # Yaw Drag-Steering Gains
        self.kp_yaw = 0.035        # Yaw steering gain per degree of azimuth error
        self.kd_yaw = 0.015        # Yaw damping gain
        self.k_gyro_yaw = 0.005    # Gyro yaw-rate feedback damping

        # Pitch Drag-Steering & Ballast Gains
        self.kp_pitch = 0.040      # Pitch steering gain per degree of elevation error
        self.kd_pitch = 0.018      # Pitch damping gain

        # State tracking
        self.prev_azimuth_err = 0.0
        self.prev_elevation_err = 0.0
        self.last_update_time = None
        self.current_mode = "SEARCH"

    def reset(self):
        self.prev_azimuth_err = 0.0
        self.prev_elevation_err = 0.0
        self.last_update_time = None
        self.current_mode = "SEARCH"

    def update(self, target_info: dict, bno_data: dict, current_time: float = None) -> tuple[dict, dict]:
        """
        Compute control commands from target relative info and BNO IMU data.
        Returns:
            cmd: Control command dictionary compatible with nodes/pc/compute.py
            status: Informative telemetry dictionary for visualization
        """
        if current_time is None:
            current_time = time.time()

        if self.last_update_time is None:
            dt = 0.02
        else:
            dt = max(0.001, min(0.1, current_time - self.last_update_time))
        self.last_update_time = current_time

        target_found = bool(target_info.get("target_found", False))
        azimuth_deg = float(target_info.get("azimuth_deg", 0.0))
        elevation_deg = float(target_info.get("elevation_deg", 0.0))
        distance_m = float(target_info.get("distance_m", 99.0))
        pop_count = int(target_info.get("pop_count", 0))
        just_popped = bool(target_info.get("just_popped", False))

        depth_m = float(bno_data.get("depth_m", 1.5))
        is_surfaced = bool(bno_data.get("is_surfaced", False) or target_info.get("is_surfaced", False))
        gyro = bno_data.get("gyro", [0.0, 0.0, 0.0])  # [gx, gy, gz] in deg/s

        # -------------------------------------------------------------
        # 1. State Machine & Mode Selection
        # -------------------------------------------------------------
        # Surface breach safety override: If surfaced or very close to surface, force dive
        if is_surfaced or depth_m < 0.35:
            self.current_mode = "SURFACE_RECOVERY"
        elif not target_found:
            self.current_mode = "SEARCH"
        elif distance_m < 1.3:
            self.current_mode = "STRIKE"
        else:
            self.current_mode = "APPROACH"

        # -------------------------------------------------------------
        # BNO Roll Orientation Compensation (Head-Up Earth-Level Projection)
        # -------------------------------------------------------------
        # Target azimuth & elevation from camera/body frame are projected onto the Earth-level horizon.
        # This precisely matches the operator stick convention (Head-up mode) expected by compute.py,
        # ensuring that pushing UP/RIGHT always deploys whichever leg is physically on TOP/RIGHT in world gravity!
        roll_deg = float(bno_data.get("roll", 0.0))
        pitch_deg = float(bno_data.get("pitch", 0.0))
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        az_rad = math.radians(azimuth_deg)
        el_rad = math.radians(elevation_deg)

        # Unit vector pointing to target in body frame (+X: Fwd, +Y: Left, +Z: Up)
        xb = math.cos(el_rad) * math.cos(az_rad)
        yb = -math.cos(el_rad) * math.sin(az_rad)
        zb = math.sin(el_rad)

        # De-roll around X-axis by +roll_rad to obtain Earth-level horizontal (Y) and vertical (Z) components
        y_level = math.cos(roll_rad) * yb - math.sin(roll_rad) * zb
        z_level = math.sin(roll_rad) * yb + math.cos(roll_rad) * zb

        azimuth_level = math.degrees(math.atan2(-y_level, max(1e-4, xb)))
        elevation_level = math.degrees(math.atan2(z_level, math.hypot(xb, y_level)))

        # Project body gyro rates [gx, gy, gz] onto Earth vertical (gravity Z) axis for pure yaw damping:
        # omega_z_world = -gx * sin(pitch) + gy * sin(roll)*cos(pitch) + gz * cos(roll)*cos(pitch)
        omega_z_world = (
            -gyro[0] * math.sin(pitch_rad)
            + gyro[1] * math.sin(roll_rad) * math.cos(pitch_rad)
            + gyro[2] * math.cos(roll_rad) * math.cos(pitch_rad)
        )

        # -------------------------------------------------------------
        # 2. Guidance Calculations per Mode
        # -------------------------------------------------------------
        if self.current_mode == "SURFACE_RECOVERY":
            # Force ballast full intake (sink) + downward pitch drag (nose down)
            throttle = 0.40
            steer_yaw = 0.0
            steer_pitch = -0.70  # Top/Bottom leg differential for nose down
            ballast = 1.0        # Max ballast intake to pull AUV underwater
            brake = False

        elif self.current_mode == "SEARCH":
            # Slow cruise and turn to sweep pool
            throttle = 0.35
            steer_yaw = 0.60     # Gentle sweeping turn
            # Maintain safe mid-water depth (around 1.8m)
            depth_err = 1.8 - depth_m
            steer_pitch = float(np.clip(depth_err * 0.5, -0.5, 0.5))
            ballast = float(np.clip(-depth_err * 0.3, -0.5, 0.5))
            brake = False

        else:
            # TARGET ACQUIRED: APPROACH or STRIKE
            # ---------------------------------------------------------
            # (A) Horizontal Yaw Drag-Steering (Earth-level azimuth error)
            # ---------------------------------------------------------
            d_az = (azimuth_level - self.prev_azimuth_err) / dt
            # PD control + Earth-level vertical gyro rate damping
            # If target is to the right in Earth level (azimuth > 0), steer_yaw > 0
            yaw_cmd = self.kp_yaw * azimuth_level + self.kd_yaw * d_az + self.k_gyro_yaw * omega_z_world
            steer_yaw = float(np.clip(yaw_cmd, -1.0, 1.0))
            self.prev_azimuth_err = azimuth_level

            # ---------------------------------------------------------
            # (B) Vertical Pitch Drag-Steering & Ballast (Earth-level elevation error)
            # ---------------------------------------------------------
            d_el = (elevation_level - self.prev_elevation_err) / dt
            # If target is above in Earth level (elevation > 0), steer_pitch > 0
            pitch_cmd = self.kp_pitch * elevation_level + self.kd_pitch * d_el
            steer_pitch = float(np.clip(pitch_cmd, -1.0, 1.0))
            self.prev_elevation_err = elevation_level

            # Ballast intake control (Earth vertical buoyancy trim):
            # When target is below in world: intake water (ballast > 0).
            # When target is above in world: purge water (ballast < 0).
            ballast_base = -0.02 * elevation_level
            # Depth safety trim: keep within pool depth (1.0m to 2.8m)
            if depth_m < 0.6:
                ballast_base += 0.4
            elif depth_m > 3.0:
                ballast_base -= 0.4
            ballast = float(np.clip(ballast_base, -1.0, 1.0))

            # ---------------------------------------------------------
            # (C) Forward BLDC Propulsion
            # ---------------------------------------------------------
            abs_az = abs(azimuth_level)
            if self.current_mode == "STRIKE":
                # Final charge: Full thrust!
                throttle = 1.0
            else:
                # Approach mode: If azimuth error is large, reduce thrust slightly to allow tight turn,
                # then accelerate once aligned.
                if abs_az > 45.0:
                    throttle = 0.55
                elif abs_az > 20.0:
                    throttle = 0.75
                else:
                    throttle = 0.90

            brake = False

        # Build output control command dict
        cmd = {
            "throttle": round(float(throttle), 3),
            "steer_yaw": round(float(steer_yaw), 3),
            "steer_pitch": round(float(steer_pitch), 3),
            "steer_roll": 0.0,
            "stick_right_x": round(float(steer_yaw), 3),
            "stick_right_y": round(float(steer_pitch), 3),
            "ballast": round(float(ballast), 3),
            "brake": brake,
            "e_stop": False,
            "mode": self.current_mode,
            # Compatibility aliases
            "vx": round(float(throttle), 3),
            "vyaw": round(float(steer_yaw), 3),
            "pitch": round(float(steer_pitch), 3),
        }

        guidance_status = {
            "mode": self.current_mode,
            "target_found": target_found,
            "azimuth_err_deg": round(azimuth_level, 2),
            "elevation_err_deg": round(elevation_level, 2),
            "azimuth_body_deg": round(azimuth_deg, 2),
            "elevation_body_deg": round(elevation_deg, 2),
            "roll_deg": round(roll_deg, 2),
            "pitch_deg": round(pitch_deg, 2),
            "omega_z_world": round(omega_z_world, 2),
            "distance_m": round(distance_m, 2),
            "pop_count": pop_count,
            "just_popped": just_popped,
            "throttle": cmd["throttle"],
            "steer_yaw": cmd["steer_yaw"],
            "steer_pitch": cmd["steer_pitch"],
            "ballast": cmd["ballast"],
            "depth_m": round(depth_m, 2),
            "is_surfaced": is_surfaced,
        }

        return cmd, guidance_status


def main():
    node = Node()
    controller = AIGuidanceController()

    target_info = {
        "target_found": False,
        "azimuth_deg": 0.0,
        "elevation_deg": 0.0,
        "distance_m": 0.0,
        "pop_count": 0,
        "just_popped": False,
        "is_surfaced": False,
    }

    bno_data = {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "gyro": [0.0, 0.0, 0.0],
        "accel": [0.0, 0.0, 9.81],
        "depth_m": 1.5,
        "is_surfaced": False,
    }

    seq = 0
    last_print_time = 0.0

    print("[AI Guidance Node] Initialized. Awaiting target_relative_info and bno_data...")

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[AI Guidance Node] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                input_id = event["id"]
                raw_value = event["value"]

                try:
                    raw_bytes = raw_value.to_pylist()[0]
                    if isinstance(raw_bytes, str):
                        parsed_json = json.loads(raw_bytes)
                    else:
                        parsed_json = json.loads(raw_bytes.decode("utf-8"))
                except Exception:
                    parsed_json = {}

                if input_id == "target_relative_info":
                    target_info.update(parsed_json)
                elif input_id == "bno_data":
                    bno_data.update(parsed_json)

                # Execute guidance calculation on tick
                if input_id == "tick":
                    seq += 1
                    now = time.time()

                    cmd, status = controller.update(target_info, bno_data, now)
                    cmd["seq"] = seq
                    cmd["timestamp"] = now

                    # 1. Publish control_cmd to pc/compute.py
                    cmd_bytes = json.dumps(cmd).encode("utf-8")
                    node.send_output("control_cmd", pa.array([cmd_bytes]))

                    # 2. Publish guidance_status for visualizer and diagnostics
                    status_bytes = json.dumps(status).encode("utf-8")
                    node.send_output("guidance_status", pa.array([status_bytes]))

                    # Periodic console logging (every ~1s)
                    if now - last_print_time >= 1.0:
                        last_print_time = now
                        print(
                            f"[AI Guidance] Mode:{status['mode']:16s} "
                            f"Dist:{status['distance_m']:5.2f}m "
                            f"Az:{status['azimuth_err_deg']:+6.1f}deg "
                            f"El:{status['elevation_err_deg']:+5.1f}deg "
                            f"Thr:{status['throttle']:.2f} "
                            f"Yaw:{status['steer_yaw']:+5.2f} "
                            f"Pitch:{status['steer_pitch']:+5.2f} "
                            f"Pops:{status['pop_count']}"
                        )

    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
