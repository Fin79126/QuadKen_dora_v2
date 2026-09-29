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
        # Leg deployment configuration
        self.max_deploy_deg = 75.0     # Maximum opening angle (0 to 90 deg range)
        self.spread_factor = 1.5       # Spread factor: 1.5 allows side legs to deploy at 50% for membrane opening

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
        stick_right_x: float = None,
        stick_right_y: float = None,
    ):
        # 1. Forward BLDC Propulsion
        # Left stick operates whole-body forward propulsion. Clamp throttle to [0.0, 1.0].
        fwd_thrust = max(0.0, min(1.0, throttle))

        if brake:
            bldc_pwm = [0, 0]
        else:
            pwm_val = int(round(fwd_thrust * 100))
            bldc_pwm = [pwm_val, pwm_val]

        # 2. Drag Steering (4 Membrane Legs: [0: Top, 1: Right, 2: Bottom, 3: Left])
        # Direct degree control (0 deg = closed/streamlined, max_deploy_deg = full deploy)
        if brake:
            # Full 4-leg umbrella deployment for hydrodynamic braking
            leg_angles = [self.max_deploy_deg] * 4
        else:
            # Parse right-stick directional vector (X: Right/Left, Y: Up/Down)
            sx = stick_right_x if stick_right_x is not None else steer_yaw
            sy = stick_right_y if stick_right_y is not None else steer_pitch

            stick_mag = min(1.0, math.hypot(sx, sy))

            # Nominal leg orientation angles in robot rear-facing projection (u: Right, v: Top):
            # Leg 0 (Top): pi/2 (+90 deg)
            # Leg 1 (Right): 0.0 (0 deg)
            # Leg 2 (Bottom): -pi/2 (-90 deg)
            # Leg 3 (Left): pi (180 deg)
            leg_nominals = [math.pi / 2.0, 0.0, -math.pi / 2.0, math.pi]
            leg_deploys = [0.0, 0.0, 0.0, 0.0]

            if stick_mag > 0.05:
                # User stick direction angle (rad) in operator/world perspective
                stick_angle = math.atan2(sy, sx)

                # BNO Orientation Compensation (Head-up mode):
                # When robot rolls by roll_deg, rotate stick angle into robot body frame so that
                # pushing the stick UP always deploys whichever leg is physically on top in world gravity!
                target_body_angle = stick_angle + math.radians(roll_deg)

                # Fan / Membrane Spread deployment:
                # Main leg deploys 100%. Side adjacent legs (at 90 deg) deploy at cos(90/1.5) = 50%
                # to fan out the membrane skins between legs for maximum steering bite.
                # Opposite legs (> 135 deg) remain fully closed.
                for i in range(4):
                    diff = math.atan2(
                        math.sin(target_body_angle - leg_nominals[i]),
                        math.cos(target_body_angle - leg_nominals[i]),
                    )
                    if abs(diff) < math.radians(135.0):
                        w = math.cos(diff / self.spread_factor)
                    else:
                        w = 0.0
                    leg_deploys[i] = stick_mag * w * self.max_deploy_deg

            # IMU Posture Compensation (stabilize pitch tilt when stick is neutral or gentle)
            if stick_mag < 0.3:
                fade = 1.0 - (stick_mag / 0.3)
                pitch_comp = float(np.clip(-pitch_deg * 0.4, -15.0, 15.0)) * fade
                if pitch_comp > 0:
                    leg_deploys[2] += pitch_comp  # Pitch down tilt -> deploy bottom leg
                else:
                    leg_deploys[0] += abs(pitch_comp)  # Pitch up tilt -> deploy top leg

            # Clamp all leg angles to [0.0, self.max_deploy_deg]
            leg_angles = [
                round(float(np.clip(leg_deploys[i], 0.0, self.max_deploy_deg)), 1)
                for i in range(4)
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
                stick_rx = control_cmd.get("stick_right_x", steer_yaw)
                stick_ry = control_cmd.get("stick_right_y", steer_pitch)
                ballast_cmd = control_cmd.get("ballast", 0.0)
                brake = control_cmd.get("brake", False)

                stick_mag = math.hypot(stick_rx, stick_ry)

                if is_estop:
                    robot_state = "EMERGENCY_STOP"
                elif not esp1_connected or not esp2_connected:
                    robot_state = "FAILSAFE_ESP_DISCONNECTED"
                elif brake:
                    robot_state = "HYDRO_BRAKING"
                elif throttle > 0.05:
                    if stick_mag > 0.1:
                        robot_state = "DRAG_STEERING"
                    else:
                        robot_state = "CRUISING"
                elif stick_mag > 0.1:
                    robot_state = "DRAG_STEERING"
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
                        stick_right_x=stick_rx,
                        stick_right_y=stick_ry,
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
