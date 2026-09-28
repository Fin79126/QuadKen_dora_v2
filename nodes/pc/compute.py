"""
PC Compute Node (QuadKen Brain)
Performs kinematic calculations, posture stabilization, and gait generation.
Subscribes to:
  - control_cmd: Target velocities and modes from pc/controller
  - bno_data: IMU orientation and inertial data from raspi/bno
  - esp_status: TCP connection states from raspi/esp_bridge
  - esp_telemetry: UDP feedback from raspi/esp_bridge
Publishes:
  - actuator_cmd: Servo angles and motor commands destined for the 2 ESPs
  - compute_status: High-level calculation state for Rerun visualizer
"""

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


class QuadrupedKinematics:
    """
    Inverse Kinematics and Gait calculation for QuadKen.
    Computes 12 joint angles (6 for ESP1, 6 for ESP2).
    """
    def __init__(self):
        self.phase = 0.0
        self.gait_frequency = 1.5  # Hz

    def update_gait(self, dt: float, vx: float, vyaw: float, height: float, roll_comp: float, pitch_comp: float):
        self.phase = (self.phase + 2.0 * math.pi * self.gait_frequency * dt) % (2.0 * math.pi)

        # Baseline stand angles (degrees)
        hip_base = 0.0
        thigh_base = 35.0
        calf_base = -65.0

        # Simple sinusoidal gait modulation for walking
        amp_swing = 15.0 * vx
        fl_leg = math.sin(self.phase) * amp_swing
        fr_leg = math.sin(self.phase + math.pi) * amp_swing
        rl_leg = math.sin(self.phase + math.pi) * amp_swing
        rr_leg = math.sin(self.phase) * amp_swing

        # ESP1 (Front Legs: FL + FR = 6 servos)
        esp1_servos = [
            hip_base + roll_comp * 10.0,
            thigh_base + fl_leg + pitch_comp * 10.0,
            calf_base - fl_leg,
            -hip_base - roll_comp * 10.0,
            thigh_base + fr_leg + pitch_comp * 10.0,
            calf_base - fr_leg,
        ]

        # ESP2 (Rear Legs: RL + RR = 6 servos)
        esp2_servos = [
            hip_base + roll_comp * 10.0,
            thigh_base + rl_leg - pitch_comp * 10.0,
            calf_base - rl_leg,
            -hip_base - roll_comp * 10.0,
            thigh_base + rr_leg - pitch_comp * 10.0,
            calf_base - rr_leg,
        ]

        # Clamp servo angles to safe range [-90, 90]
        esp1_servos = [round(float(np.clip(a, -90.0, 90.0)), 2) for a in esp1_servos]
        esp2_servos = [round(float(np.clip(a, -90.0, 90.0)), 2) for a in esp2_servos]

        # Auxiliary motor speeds (-100 to 100 PWM)
        esp1_motors = [int(np.clip(vx * 100, -100, 100)), int(np.clip(vx * 100, -100, 100))]
        esp2_motors = [int(np.clip(vx * 100, -100, 100)), int(np.clip(vx * 100, -100, 100))]

        return esp1_servos, esp1_motors, esp2_servos, esp2_motors


def main():
    node = Node()
    kinematics = QuadrupedKinematics()

    # Cached states
    control_cmd = {
        "vx": 0.0, "vy": 0.0, "vyaw": 0.0, "body_height": 0.25,
        "roll": 0.0, "pitch": 0.0, "gait_mode": 1, "e_stop": False
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

    for event in node:
        event_type = event["type"]
        if event_type == "STOP":
            print("[Compute] Received STOP event. Exiting.")
            sys.exit(0)

        if event_type == "INPUT":
            input_id = event["id"]
            raw_value = event["value"]

            try:
                # Value can be Arrow array of strings/bytes
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

            # Compute loop triggered by control_cmd or timer
            now = time.time()
            dt = max(0.005, min(0.1, now - last_time))
            last_time = now

            # Check E-Stop condition (software e-stop or ESP disconnected)
            is_estop = control_cmd.get("e_stop", False)
            esp1_connected = esp_status.get("esp1", {}).get("connected", False)
            esp2_connected = esp_status.get("esp2", {}).get("connected", False)

            if is_estop:
                robot_state = "EMERGENCY_STOP"
            elif not esp1_connected or not esp2_connected:
                robot_state = "FAILSAFE_ESP_DISCONNECTED"
            elif control_cmd.get("gait_mode", 0) == 0:
                robot_state = "STAND"
            else:
                robot_state = "WALKING"

            # Posture compensation from IMU
            roll_comp = -float(bno_data.get("roll", 0.0)) * 0.2
            pitch_comp = -float(bno_data.get("pitch", 0.0)) * 0.2

            if robot_state in ("EMERGENCY_STOP", "FAILSAFE_ESP_DISCONNECTED"):
                # Neutral safe position with motors disabled
                esp1_servos = [0.0] * 6
                esp1_motors = [0, 0]
                esp2_servos = [0.0] * 6
                esp2_motors = [0, 0]
            else:
                esp1_servos, esp1_motors, esp2_servos, esp2_motors = kinematics.update_gait(
                    dt=dt,
                    vx=control_cmd.get("vx", 0.0),
                    vyaw=control_cmd.get("vyaw", 0.0),
                    height=control_cmd.get("body_height", 0.25),
                    roll_comp=roll_comp,
                    pitch_comp=pitch_comp,
                )

            seq += 1
            actuator_cmd = {
                "seq": seq,
                "timestamp": now,
                "robot_state": robot_state,
                "esp1": {
                    "servos": esp1_servos,
                    "motors": esp1_motors,
                },
                "esp2": {
                    "servos": esp2_servos,
                    "motors": esp2_motors,
                },
            }

            compute_status = {
                "seq": seq,
                "robot_state": robot_state,
                "phase": round(float(kinematics.phase), 2),
                "dt_ms": round(float(dt * 1000.0), 2),
                "esp1_connected": esp1_connected,
                "esp2_connected": esp2_connected,
            }

            # Send output to dora network
            node.send_output("actuator_cmd", pa.array([json.dumps(actuator_cmd).encode("utf-8")]))
            node.send_output("compute_status", pa.array([json.dumps(compute_status).encode("utf-8")]))


if __name__ == "__main__":
    main()
