"""
Test script for QuadKen Full Autonomous Balloon Destruction (Step 4 Verification).
Integrates:
  - QuadKenMuJoCoSim (Physics SITL + Front Underwater Camera)
  - BalloonDetector (OpenCV Underwater Perception)
  - AIGuidanceController (GNC: Search, Approach, Strike)
  - UnderwaterDynamics (Leg & Ballast Kinematic Allocation)

Verifies full perception-in-the-loop autonomous guidance without using ground-truth
balloon coordinates for steering!
"""

import os
import sys
import time
import math
import numpy as np

# Ensure workspace root is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim
from nodes.pc.balloon_detector import BalloonDetector
from nodes.pc.ai_guidance import AIGuidanceController
from nodes.pc.compute import UnderwaterDynamics


def run_full_autonomous_test(target_pops: int = 3, max_sim_seconds: float = 80.0):
    print("=" * 70)
    print(" [TEST] Starting QuadKen Step 4 Perception-Driven Full Autonomous Test")
    print("=" * 70)

    # Initialize simulation with random spawn disabled for reproducible test first,
    # or with random_spawn=False (uses fixed waypoints).
    sim = QuadKenMuJoCoSim(random_spawn=False)
    detector = BalloonDetector()
    guidance = AIGuidanceController()
    dynamics = UnderwaterDynamics()

    dt = 0.02  # 50 Hz physics step
    total_steps = int(max_sim_seconds / dt)

    pop_times = []
    print(f"Total initial balloons: {len(sim.balloon_waypoints)}")
    print(f"Target pops needed: {target_pops}")

    last_perception_target_found = False

    for step in range(total_steps):
        current_time = step * dt

        # 1. Physics & Sensor Telemetry
        bno_data = sim.get_bno_payload(step)
        gt_info = sim.get_target_relative_info()

        # 2. Render front underwater camera (Raw BGR for OpenCV perception)
        raw_bgr = sim.render_front_camera(return_raw_bgr=True)

        # 3. Step 3 Underwater Perception (OpenCV)
        # Note: ONLY camera image and optional bno_data are given to the detector!
        vision_info, annotated_bgr = detector.detect(raw_bgr, bno_data)

        # Check pop event from sim
        pop_cnt = sim.balloon_pop_count
        if pop_cnt > len(pop_times):
            pop_times.append(current_time)
            print(f"\n>>> [PERCEPTION HIT #{len(pop_times)}] at t={current_time:.2f}s! (Total Pops: {pop_cnt}) <<<")
            if len(pop_times) >= target_pops:
                print(f"\n[TEST SUCCESS] Achieved {len(pop_times)} perception-driven balloon destructions in {current_time:.2f}s!")
                break

        # 4. GNC Step: Compute guidance using ONLY perception info + BNO!
        # Do NOT pass gt_info to guidance!
        # We inject pop_count from bno/sim so telemetry matches, but control angles come strictly from vision!
        vision_info["pop_count"] = pop_cnt
        cmd, status = guidance.update(vision_info, bno_data, current_time)

        # 5. Actuator Dynamics Allocation
        leg_angles, bldc_pwm, ballast_servos = dynamics.compute_actuators(
            dt=dt,
            throttle=cmd["throttle"],
            steer_yaw=cmd["steer_yaw"],
            steer_pitch=cmd["steer_pitch"],
            steer_roll=cmd["steer_roll"],
            ballast_cmd=cmd["ballast"],
            brake=cmd["brake"],
            roll_deg=bno_data["roll"],
            pitch_deg=bno_data["pitch"],
            stick_right_x=cmd["stick_right_x"],
            stick_right_y=cmd["stick_right_y"],
        )

        # 6. Apply to simulation
        actuator_cmd = {
            "esp1": {
                "motors": bldc_pwm,
                "servos": leg_angles,
            },
            "esp2": {
                "servos": ballast_servos,
            }
        }
        sim.set_actuator_commands(actuator_cmd)

        # 7. Step physics
        sim.step()

        # Telemetry logging every 50 steps (1s)
        if step % 50 == 0:
            found_str = "VIS:FOUND" if vision_info["target_found"] else "VIS:SEARCH"
            p = sim.data.qpos[0:3]
            v = sim.data.qvel[0:3]
            print(
                f"[t={current_time:5.1f}s] {found_str:10s} Mode:{status['mode']:16s} "
                f"Pos:[{p[0]:5.2f},{p[1]:5.2f},{p[2]:5.2f}] Vel:[{v[0]:4.2f},{v[1]:4.2f},{v[2]:4.2f}] "
                f"Dist:{vision_info.get('distance_m', 0.0):4.2f}m "
                f"Az:{vision_info.get('azimuth_deg', 0.0):+5.1f} "
                f"Thr:{status['throttle']:.2f} Yaw:{status['steer_yaw']:+4.2f} "
                f"Pch:{status['steer_pitch']:+4.2f} Bal:{status['ballast']:+4.2f} Pops:{pop_cnt}"
            )

    print("=" * 70)
    print(f"[TEST FINISHED] Total Pops: {sim.balloon_pop_count}, Total Simulated Time: {step * dt:.1f}s")
    print("=" * 70)
    return sim.balloon_pop_count, pop_times


if __name__ == "__main__":
    run_full_autonomous_test()
