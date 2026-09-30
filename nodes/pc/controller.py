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
# Switch Pro Controller requires HIDAPI Switch driver to correctly parse input packets
# and prevent IMU/gyro telemetry from triggering ghost button presses (e.g. E-Stop flickering).
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH"] = "1"
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH_PLAYER_LED"] = "0"
os.environ["SDL_JOYSTICK_HIDAPI_SWITCH_HOME_LED"] = "0"
import pygame


def init_joystick():
    try:
        if not pygame.get_init():
            pygame.init()
        if not pygame.joystick.get_init():
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
    prev_estop_btn = False
    seq = 0

    try:
        for event in node:
            event_type = event["type"]
            if event_type == "STOP":
                print("[Controller] Received STOP event. Exiting.")
                break

            if event_type == "INPUT":
                # Handle hotplugging events
                for pg_event in pygame.event.get():
                    if pg_event.type == pygame.JOYDEVICEADDED:
                        if joystick is None:
                            print("[Controller] Gamepad plugged in. Initializing...")
                            joystick = init_joystick()
                    elif pg_event.type == pygame.JOYDEVICEREMOVED:
                        print("[Controller] Gamepad disconnected. Falling back to simulated mode.")
                        if joystick is not None:
                            try:
                                joystick.quit()
                            except Exception:
                                pass
                            joystick = None

                # Periodic reconnect check if no gamepad is active (every ~2s at 100ms ticks)
                if joystick is None and seq % 20 == 0:
                    if pygame.joystick.get_count() > 0:
                        joystick = init_joystick()

                raw_axes = []

                if joystick is not None:
                    try:
                        num_axes = joystick.get_numaxes()
                        raw_axes = [round(float(joystick.get_axis(i)), 3) for i in range(num_axes)]

                        # Gamepad mapping:
                        # Left Stick: Whole-body propulsion ONLY (Y-axis: forward thrust 0.0 to 1.0)
                        # Left stick horizontal tilt (raw_x) is intentionally ignored.
                        raw_y = -raw_axes[1] if num_axes > 1 else 0.0
                        raw_rx = raw_axes[2] if num_axes > 2 else 0.0
                        raw_ry = -raw_axes[3] if num_axes > 3 else 0.0

                        deadband = 0.08
                        # Forward-only propulsion: clamp negative (backward) to 0.0
                        throttle = max(0.0, raw_y) if abs(raw_y) > deadband else 0.0

                        # Right Stick: Drag Steering & Directional Leg deployment
                        # rx: Right (+1.0) / Left (-1.0)
                        # ry: Up (+1.0) / Down (-1.0)
                        rx = raw_rx if abs(raw_rx) > deadband else 0.0
                        ry = raw_ry if abs(raw_ry) > deadband else 0.0

                        steer_yaw = rx
                        steer_pitch = ry

                        # Buttons:
                        # Button 0 (A): Toggle E-Stop (edge-triggered)
                        curr_estop = bool(joystick.get_button(0)) if joystick.get_numbuttons() > 0 else False
                        if curr_estop and not prev_estop_btn:
                            e_stop = not e_stop
                            print(f"[Controller] E-Stop toggled: {e_stop}")
                        prev_estop_btn = curr_estop

                        # Button 2 (X): Water Brake (deploy all legs)
                        brake = bool(joystick.get_button(2)) if joystick.get_numbuttons() > 2 else False

                        # Triggers or Shoulder buttons for Ballast intake/purge
                        # Compatible with Switch Pro Controller (R=10, ZR=17, L=9, ZL=16, D-pad),
                        # and Xbox/Standard controllers (RB=5, LB=4, D-pad Hat, Analog Triggers).
                        num_buttons = joystick.get_numbuttons()
                        num_axes = joystick.get_numaxes()

                        intake_pressed = False
                        purge_pressed = False

                        # 1. Check Shoulder / Trigger buttons
                        # R / ZR / RB
                        for btn_idx in [10, 17, 5]:
                            if btn_idx < num_buttons and joystick.get_button(btn_idx):
                                intake_pressed = True
                                break

                        # L / ZL / LB
                        for btn_idx in [9, 16, 4]:
                            if btn_idx < num_buttons and joystick.get_button(btn_idx):
                                purge_pressed = True
                                break

                        # 2. Check D-pad buttons / Hats (Up: surface, Down: dive)
                        if not intake_pressed and num_buttons > 12 and joystick.get_button(12):  # D-pad Down
                            intake_pressed = True
                        if not purge_pressed and num_buttons > 11 and joystick.get_button(11):  # D-pad Up
                            purge_pressed = True

                        if joystick.get_numhats() > 0:
                            _, hat_y = joystick.get_hat(0)
                            if hat_y < 0:
                                intake_pressed = True
                            elif hat_y > 0:
                                purge_pressed = True

                        # 3. Check Analog Triggers (LT/RT) if present
                        if not intake_pressed and num_axes > 5 and joystick.get_axis(5) > 0.4:
                            intake_pressed = True
                        if not purge_pressed and num_axes > 4 and joystick.get_axis(4) > 0.4:
                            purge_pressed = True

                        # Velocity command for ballast fill ratio:
                        # +1.0: filling water (diving), -1.0: purging water (surfacing), 0.0: hold
                        if intake_pressed and not purge_pressed:
                            ballast = 1.0
                        elif purge_pressed and not intake_pressed:
                            ballast = -1.0
                        else:
                            ballast = 0.0
                    except pygame.error as pe:
                        print(f"[Controller] Gamepad communication error ({pe}). Falling back to simulated mode.")
                        if joystick is not None:
                            try:
                                joystick.quit()
                            except Exception:
                                pass
                        joystick = None
                else:
                    # Simulated dynamic underwater cruising demonstration
                    t = time.time()
                    # Forward cruising with periodic turns and ballast breathing
                    throttle = 0.4 + 0.2 * np.sin(0.3 * t)
                    steer_yaw = 0.5 * np.sin(0.4 * t)
                    steer_pitch = 0.2 * np.cos(0.2 * t)
                    ballast = 0.3 * np.sin(0.15 * t)
                    brake = False
                    raw_axes = [0.0, round(float(throttle), 3), round(float(steer_yaw), 3), round(float(steer_pitch), 3)]
                    rx = steer_yaw
                    ry = steer_pitch

                seq += 1
                cmd_payload = {
                    "seq": seq,
                    "timestamp": time.time(),
                    "throttle": round(float(throttle), 3),
                    "steer_yaw": round(float(steer_yaw), 3),
                    "steer_pitch": round(float(steer_pitch), 3),
                    "steer_roll": round(float(steer_roll), 3),
                    "stick_right_x": round(float(rx), 3),
                    "stick_right_y": round(float(ry), 3),
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
        if joystick is not None:
            try:
                joystick.quit()
            except Exception:
                pass
        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main()
