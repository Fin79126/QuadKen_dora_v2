"""
PC Controller Node
Reads joystick/gamepad inputs using Pygame.
Provides keyboard / simulated fallback when no physical joystick is detected.
Publishes:
  - control_cmd: Command velocities, pose adjustments, gait mode, and E-Stop.
"""

import time
import json
import os
import sys
import numpy as np
import pyarrow as pa
from dora import Node

# Optional OpenTelemetry tracing
try:
    from opentelemetry import trace
    tracer = trace.get_tracer("quadken.pc.controller")
except ImportError:
    tracer = None

# Suppress Pygame welcome banner
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import pygame


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if joystick_count > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        print(f"[Controller] Gamepad detected: {js.get_name()}")
        return js
    else:
        print("[Controller] No physical gamepad detected. Using simulated/keyboard fallback mode.")
        return None


def main():
    node = Node()
    joystick = init_joystick()

    # Controller state variables
    vx = 0.0          # Forward velocity (-1.0 to 1.0)
    vy = 0.0          # Lateral velocity (-1.0 to 1.0)
    vyaw = 0.0        # Turn velocity (-2.0 to 2.0)
    body_height = 0.25 # Body nominal height (m)
    roll = 0.0
    pitch = 0.0
    gait_mode = 1     # 0: IDLE/STAND, 1: WALK, 2: TROT
    e_stop = False
    seq = 0

    rate_hz = 50
    dt = 1.0 / rate_hz

    # Check if dataflow drives us via timer input, or we drive the loop
    for event in node:
        event_type = event["type"]
        if event_type == "STOP":
            print("[Controller] Received STOP event. Exiting.")
            sys.exit(0)

        if event_type == "INPUT":
            # Can be triggered by timer tick (e.g. dora/timer/millis/20)
            input_id = event["id"]

        # Read Joystick inputs
        pygame.event.pump()
        if joystick is not None:
            # Typical Xbox/Playstation mapping
            # Axis 0: Left Stick X, Axis 1: Left Stick Y (inverted)
            # Axis 2: Right Stick X, Axis 3: Right Stick Y
            raw_vx = -joystick.get_axis(1) if joystick.get_numaxes() > 1 else 0.0
            raw_vy = joystick.get_axis(0) if joystick.get_numaxes() > 0 else 0.0
            raw_vyaw = joystick.get_axis(2) if joystick.get_numaxes() > 2 else 0.0
            raw_pitch = -joystick.get_axis(3) if joystick.get_numaxes() > 3 else 0.0

            # Deadband threshold
            deadband = 0.08
            vx = raw_vx if abs(raw_vx) > deadband else 0.0
            vy = raw_vy if abs(raw_vy) > deadband else 0.0
            vyaw = raw_vyaw if abs(raw_vyaw) > deadband else 0.0
            pitch = raw_pitch * 0.3 if abs(raw_pitch) > deadband else 0.0

            # Check buttons
            # Button 0 (A/Cross): Toggle Gait
            if joystick.get_numbuttons() > 0 and joystick.get_button(0):
                gait_mode = (gait_mode + 1) % 3
            # Button 1 (B/Circle): E-Stop toggle
            if joystick.get_numbuttons() > 1 and joystick.get_button(1):
                e_stop = not e_stop
        else:
            # Simulated cyclic demonstration input when no physical joystick is attached
            t = time.time()
            vx = 0.3 * np.sin(0.5 * t)
            vy = 0.0
            vyaw = 0.2 * np.cos(0.3 * t)
            pitch = 0.05 * np.sin(0.8 * t)

        seq += 1
        cmd_payload = {
            "seq": seq,
            "timestamp": time.time(),
            "vx": round(float(vx), 3),
            "vy": round(float(vy), 3),
            "vyaw": round(float(vyaw), 3),
            "body_height": round(float(body_height), 3),
            "roll": round(float(roll), 3),
            "pitch": round(float(pitch), 3),
            "gait_mode": int(gait_mode),
            "e_stop": bool(e_stop),
        }

        # Send control command to dora network
        payload_bytes = json.dumps(cmd_payload).encode("utf-8")
        node.send_output("control_cmd", pa.array([payload_bytes]))


if __name__ == "__main__":
    main()
