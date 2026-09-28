"""
PC Compute Node (QuadKen Underwater Brain)
Performs:
  - Drag-based Steering kinematics (4 membrane-connected legs for Yaw/Pitch control)
  - Forward BLDC Propulsion allocation (2x synchronized forward thrusters)
  - Buoyancy & Ballast control (4x head water-intake servos)
  - IMU posture stabilization & Failsafe monitoring
Subscribes to:
  - control_cmd: Throttle, steering, ballast, and brake from pc/controller
  - bno_data: Underwater orientation and IMU readings from raspi/bno
  - esp_status: TCP connection states from raspi/esp_bridge
  - esp_telemetry: UDP sensor feedback from raspi/esp_bridge
Publishes:
  - actuator_cmd: ESP1 (4 leg servos + 2 BLDC) & ESP2 (4 ballast servos)
  - compute_status: High-level calculation state for Rerun visualizer
"""

import sys
import time
import json
import math
import numpy as np
import pyarrow as pa
from dora import Node

# Optional OpenTelemetry tracing
try:
    from opentelemetry import trace
    tracer = trace.get_tracer("quadken.pc.compute")
except ImportError:
    tracer = None


class UnderwaterDynamics:
    """
    Kinematics and Hydrodynamics controller for QuadKen AUV.
    
    Actuators:
      - ESP1:
          * 4 Servos: Membrane leg deployment angles [Leg1:Top, Leg2:Right, Leg3:Bottom, Leg4:Left]
            (0 deg = fully closed/streamlined, 75 deg = maximum deployed drag)
          * 2 Motors: BLDC forward thrusters [BLDC_L, BLDC_R] (0 to 100% PWM)
      - ESP2:
          * 4 Servos: Head water-intake ballast servos [Ballast_1, Ballast_2, Ballast_3, Ballast_4]
            (0 deg = closed/sealed, 90 deg = full intake opening)
    """

    def __init__(self):
        # Current ballast intake level (0.0: Empty/Positive Buoyancy, 1.0: Full/Negative Buoyancy)
        self.ballast_fill_ratio = 0.5  # Start at neutral buoyancy

    def compute_actuators(
        self,
        dt: float,
        throttle: float,
        steer_yaw: float,
        steer_pitch: float,
        steer_roll: float,
        ballast_cmd: float,
        brake: bool,
        roll_deg: float,
        pitch_deg: float,
    ):
        # 1. Forward BLDC Propulsion
        # Robot only moves forward. Clamp throttle to [0.0, 1.0].
        fwd_thrust = max(0.0, min(1.0, throttle))

        if brake:
            bldc_pwm = [0, 0]
        else:
            pwm_val = int(round(fwd_thrust * 100))
            bldc_pwm = [pwm_val, pwm_val]

        # 2. Drag Steering (4 Membrane Legs: [0: Top, 1: Right, 2: Bottom, 3: Left])
        # Opening a leg catches water flow, creating drag that turns the robot.
        max_deploy_deg = 70.0

        if brake:
            # Full umbrella deployment for hydrodynamic braking
            leg_angles = [max_deploy_deg] * 4
        else:
            # Base angle: slightly streamlined when moving forward
            leg_top = 0.0
            leg_right = 0.0
            leg_bottom = 0.0
            leg_left = 0.0

            # Yaw steering via differential drag (Right leg turns right, Left leg turns left)
            if steer_yaw > 0.05:
                # Turn Right -> Deploy Right leg (index 1)
                leg_right += steer_yaw * max_deploy_deg
            elif steer_yaw < -0.05:
                # Turn Left -> Deploy Left leg (index 3)
                leg_left += (-steer_yaw) * max_deploy_deg

            # Pitch steering via differential drag (Top leg dives down, Bottom leg pitches up)
            if steer_pitch > 0.05:
                # Pitch Up / Ascend -> Deploy Bottom leg (index 2)
                leg_bottom += steer_pitch * max_deploy_deg
            elif steer_pitch < -0.05:
                # Pitch Down / Dive -> Deploy Top leg (index 0)
                leg_top += (-steer_pitch) * max_deploy_deg

            # IMU Posture Compensation (stabilize unwanted roll and pitch tilts)
            # If robot is pitching down (pitch_deg < 0), slightly deploy bottom leg
            pitch_compensation = float(np.clip(-pitch_deg * 0.5, -20.0, 20.0))
            if pitch_compensation > 0:
                leg_bottom += pitch_compensation
            else:
                leg_top += abs(pitch_compensation)

            # Clamp all leg angles to [0.0, max_deploy_deg]
            leg_angles = [
                round(float(np.clip(leg_top, 0.0, max_deploy_deg)), 1),
                round(float(np.clip(leg_right, 0.0, max_deploy_deg)), 1),
                round(float(np.clip(leg_bottom, 0.0, max_deploy_deg)), 1),
                round(float(np.clip(leg_left, 0.0, max_deploy_deg)), 1),
            ]

        # 3. Head Water-Intake Ballast Servos (ESP2)
        # 4 servos at the cylindrical nose adjust internal water intake
        # ballast_cmd: -1.0 (purge/surface) to +1.0 (intake/dive)
        self.ballast_fill_ratio = float(
            np.clip(self.ballast_fill_ratio + ballast_cmd * 0.2 * dt, 0.0, 1.0)
        )
        ballast_angle = round(float(self.ballast_fill_ratio * 90.0), 1)
        ballast_servos = [ballast_angle] * 4

        return leg_angles, bldc_pwm, ballast_servos


def main():
    node = Node()
    dynamics = UnderwaterDynamics()

    # Cached states
    control_cmd = {
        "throttle": 0.0, "steer_yaw": 0.0, "steer_pitch": 0.0, "steer_roll": 0.0,
        "ballast": 0.0, "brake": False, "e_stop": False,
        "vx": 0.0, "vyaw": 0.0, "pitch": 0.0
    }
    bno_data = {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.0, 9.81]
    }
    esp_status = {
        "esp1": {"connected": True, "latency_ms": 1.2, "state": "READY"},
        "esp2": {"connected": True, "latency_ms": 1.5, "state": "READY"},
    }
    esp_telemetry = {}

    last_time = time.time()
    seq = 0

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[Compute] Received STOP event. Exiting.")
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

                if input_id == "control_cmd":
                    control_cmd.update(parsed_json)
                elif input_id == "bno_data":
                    bno_data.update(parsed_json)
                elif input_id == "esp_status":
                    esp_status.update(parsed_json)
                elif input_id == "esp_telemetry":
                    esp_telemetry.update(parsed_json)

                # Execute computation on 50Hz clock / at least 20ms interval
                now = time.time()
                if input_id != "bno_data" and (now - last_time < 0.02):
                    continue

                dt = max(0.005, min(0.1, now - last_time))
                last_time = now

                # Failsafe and State machine
                is_estop = control_cmd.get("e_stop", False)
                esp1_connected = esp_status.get("esp1", {}).get("connected", False)
                esp2_connected = esp_status.get("esp2", {}).get("connected", False)

                throttle = control_cmd.get("throttle", control_cmd.get("vx", 0.0))
                steer_yaw = control_cmd.get("steer_yaw", control_cmd.get("vyaw", 0.0))
                steer_pitch = control_cmd.get("steer_pitch", control_cmd.get("pitch", 0.0))
                steer_roll = control_cmd.get("steer_roll", 0.0)
                ballast_cmd = control_cmd.get("ballast", 0.0)
                brake = control_cmd.get("brake", False)

                if is_estop:
                    robot_state = "EMERGENCY_STOP"
                elif not esp1_connected or not esp2_connected:
                    robot_state = "FAILSAFE_ESP_DISCONNECTED"
                elif brake:
                    robot_state = "HYDRO_BRAKING"
                elif throttle > 0.05:
                    if abs(steer_yaw) > 0.1:
                        robot_state = "DRAG_TURNING"
                    elif abs(steer_pitch) > 0.1:
                        robot_state = "DRAG_PITCHING"
                    else:
                        robot_state = "CRUISING"
                else:
                    robot_state = "IDLE_HOVER"

                # Safety overrides
                if robot_state in ("EMERGENCY_STOP", "FAILSAFE_ESP_DISCONNECTED"):
                    # Stop BLDC, purge ballast for emergency ascent, deploy legs for drift stability
                    esp1_servos = [45.0] * 4
                    esp1_motors = [0, 0]
                    esp2_servos = [0.0] * 4  # Close water intake (purge/surface)
                else:
                    esp1_servos, esp1_motors, esp2_servos = dynamics.compute_actuators(
                        dt=dt,
                        throttle=throttle,
                        steer_yaw=steer_yaw,
                        steer_pitch=steer_pitch,
                        steer_roll=steer_roll,
                        ballast_cmd=ballast_cmd,
                        brake=brake,
                        roll_deg=float(bno_data.get("roll", 0.0)),
                        pitch_deg=float(bno_data.get("pitch", 0.0)),
                    )

                seq += 1
                actuator_cmd = {
                    "seq": seq,
                    "timestamp": now,
                    "robot_state": robot_state,
                    "esp1": {
                        "name": "Thrust_Steer",
                        "servos": esp1_servos,   # [Top, Right, Bottom, Left] deploy angles
                        "motors": esp1_motors,   # [BLDC_1, BLDC_2] forward thrust PWM
                    },
                    "esp2": {
                        "name": "Ballast",
                        "servos": esp2_servos,   # 4 head water intake servos
                        "motors": [],
                    },
                }

                compute_status = {
                    "seq": seq,
                    "robot_state": robot_state,
                    "throttle_pct": esp1_motors[0] if esp1_motors else 0,
                    "leg_deploy_angles": esp1_servos,
                    "ballast_intake_deg": esp2_servos[0] if esp2_servos else 0.0,
                    "ballast_fill_ratio": round(float(dynamics.ballast_fill_ratio), 2),
                    "dt_ms": round(float(dt * 1000.0), 2),
                    "esp1_connected": esp1_connected,
                    "esp2_connected": esp2_connected,
                }

                # Send outputs to dora network
                node.send_output("actuator_cmd", pa.array([json.dumps(actuator_cmd).encode("utf-8")]))
                node.send_output("compute_status", pa.array([json.dumps(compute_status).encode("utf-8")]))
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
