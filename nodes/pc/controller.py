"""
PC Controller Node (QuadKen Underwater AUV)
Reads joystick/gamepad inputs using Pygame.
Provides simulated fallback when no physical joystick is detected.
Controls:
  - Forward Propulsion: 2x BLDC Motors (Forward only: 0.0 to 1.0)
  - Drag Steering: 4 Membrane-connected Legs (Yaw/Pitch/Roll via water resistance)
  - Buoyancy / Ballast: 4 Head Water-Intake Servos
Publishes:
  - control_cmd: Throttle, steering angles, ballast intake level, brake, and E-Stop.
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
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH"] = "1"
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH_PLAYER_LED"] = "0"
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH_HOME_LED"] = "0"
import pygame


def init_joystick():
    try:
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            js = pygame.joystick.Joystick(0)
            js.init()
            print(f"[Controller] Gamepad detected: {js.get_name()}")
            return js
        else:
            print("[Controller] No physical gamepad detected. Using simulated AUV cruising mode.")
            return None
    except Exception as e:
        print(f"[Controller] Gamepad initialization failed ({e}). Falling back to simulated mode.")
        return None


def main():
    joystick = init_joystick()
    node = Node()

    # Underwater Controller state
    throttle = 0.0      # Forward thrust (0.0 to 1.0, forward only)
    steer_yaw = 0.0     # Turn steering via drag (-1.0 to 1.0: Left/Right)
    steer_pitch = 0.0   # Pitch angle via drag (-1.0 to 1.0: Down/Up)
    steer_roll = 0.0    # Roll angle via drag (-1.0 to 1.0)
    ballast = 0.0       # Ballast intake (-1.0: Purge/Float, 0.0: Neutral, 1.0: Intake/Sink)
    brake = False       # Full 4-leg deployment for water resistance braking
    e_stop = False      # Emergency stop
    seq = 0

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[Controller] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                pygame.event.pump()
                _ = pygame.event.get()
                raw_axes = []

                if joystick is not None:
                    num_axes = joystick.get_numaxes()
                    raw_axes = [round(float(joystick.get_axis(i)), 3) for i in range(num_axes)]

                    # Typical gamepad mapping:
                    # Axis 1: Left Stick Y -> Throttle (forward-only: 0.0 to 1.0)
                    raw_y = -raw_axes[1] if num_axes > 1 else 0.0
                    raw_x = raw_axes[0] if num_axes > 0 else 0.0
                    raw_rx = raw_axes[2] if num_axes > 2 else 0.0
                    raw_ry = -raw_axes[3] if num_axes > 3 else 0.0

                    deadband = 0.08
                    # Forward-only propulsion: clamp negative (backward) to 0.0
                    throttle = max(0.0, raw_y) if abs(raw_y) > deadband else 0.0
                    steer_yaw = raw_rx if abs(raw_rx) > deadband else (raw_x if abs(raw_x) > deadband else 0.0)
                    steer_pitch = raw_ry if abs(raw_ry) > deadband else 0.0

                    # Buttons:
                    # Button 0 (A): Toggle E-Stop
                    if joystick.get_numbuttons() > 0 and joystick.get_button(0):
                        e_stop = not e_stop
                    # Button 2 (X): Water Brake (deploy all legs)
                    brake = bool(joystick.get_button(2)) if joystick.get_numbuttons() > 2 else False

                    # Triggers or Shoulder buttons for Ballast intake/purge
                    if joystick.get_numbuttons() > 5:
                        intake_btn = joystick.get_button(5)  # RB: Intake water (dive)
                        purge_btn = joystick.get_button(4)   # LB: Purge water (surface)
                        if intake_btn and not purge_btn:
                            ballast = min(1.0, ballast + 0.05)
                        elif purge_btn and not intake_btn:
                            ballast = max(-1.0, ballast - 0.05)
                else:
                    # Simulated dynamic underwater cruising demonstration
                    t = time.time()
                    # Forward cruising with periodic turns and ballast breathing
                    throttle = 0.4 + 0.2 * np.sin(0.3 * t)
                    steer_yaw = 0.5 * np.sin(0.4 * t)
                    steer_pitch = 0.2 * np.cos(0.2 * t)
                    ballast = 0.3 * np.sin(0.15 * t)
                    brake = False
                    raw_axes = [round(float(steer_yaw), 3), round(float(throttle), 3)]

                seq += 1
                cmd_payload = {
                    "seq": seq,
                    "timestamp": time.time(),
                    "throttle": round(float(throttle), 3),
                    "steer_yaw": round(float(steer_yaw), 3),
                    "steer_pitch": round(float(steer_pitch), 3),
                    "steer_roll": round(float(steer_roll), 3),
                    "ballast": round(float(ballast), 3),
                    "brake": bool(brake),
                    "e_stop": bool(e_stop),
                    # Compatibility aliases
                    "vx": round(float(throttle), 3),
                    "vyaw": round(float(steer_yaw), 3),
                    "pitch": round(float(steer_pitch), 3),
                    "raw_axes": raw_axes,
                }

                # Send control command to dora network
                payload_bytes = json.dumps(cmd_payload).encode("utf-8")
                node.send_output("control_cmd", pa.array([payload_bytes]))
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)


if __name__ == "__main__":
    main()
