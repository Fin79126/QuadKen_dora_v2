"""
Test script for QuadKen AI Guidance (Step 2 GNC verification).
Imports AIGuidanceController directly from nodes.pc.ai_guidance and tests it
against QuadKenMuJoCoSim and UnderwaterDynamics to verify automatic balloon
tracking, approach, and popping across multiple waypoints.
"""

import os
import sys
import time
import math
import numpy as np

# Ensure workspace root is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim
from nodes.pc.compute import UnderwaterDynamics
from nodes.pc.ai_guidance import AIGuidanceController


def run_guidance_test(target_pops: int = 3, max_sim_seconds: float = 60.0):
    print("=" * 65)
    print(" [TEST] Starting QuadKen Step 2 Autonomous Guidance Simulation")
    print("=" * 65)

    sim = QuadKenMuJoCoSim()
    dynamics = UnderwaterDynamics()
    guidance = AIGuidanceController()

    dt = 0.02  # 50 Hz physics step
    total_steps = int(max_sim_seconds / dt)

    pop_times = []
    print(f"Target waypoints count: {len(sim.balloon_waypoints)}")
    print(f"Initial balloon pos: {sim.balloon_waypoints[0]}")

    for step in range(total_steps):
        current_time = step * dt

        # 1. Get telemetry from simulation
        target_info = sim.get_target_relative_info()
        bno_data = sim.get_bno_payload(step)

        # 2. Check if a pop occurred by watching pop_count
        pop_cnt = target_info.get("pop_count", 0)
        if pop_cnt > len(pop_times):
            pop_times.append(current_time)
            print(f"\n>>> [HIT #{len(pop_times)}] at t={current_time:.2f}s! Balloon #{pop_cnt} Popped! Target next: {sim.balloon_waypoints[sim.current_waypoint_idx]} <<<")
            if len(pop_times) >= target_pops:
                print(f"\n[TEST SUCCESS] Achieved {len(pop_times)} distinct balloon destructions in {current_time:.2f}s!")
                break

        # 3. Compute autonomous guidance
        cmd, status = guidance.update(target_info, bno_data, current_time)

        # 4. Compute actuator kinematics
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

        # 5. Send to MuJoCo sim
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

        # 6. Step physics
        sim.step()

        # Telemetry log every 1 second (50 steps)
        if step % 50 == 0:
            print(
                f"[t={current_time:5.1f}s] Mode:{status['mode']:16s} "
                f"Dist:{status['distance_m']:5.2f}m Az:{status['azimuth_err_deg']:+6.1f}deg "
                f"El:{status['elevation_err_deg']:+5.1f}deg Thr:{status['throttle']:.2f} "
                f"Yaw:{status['steer_yaw']:+5.2f} Pitch:{status['steer_pitch']:+5.2f} "
                f"Depth:{status['depth_m']:4.2f}m Legs:{[int(x) for x in leg_angles]}"
            )

    print("=" * 65)
    print(f"[TEST FINISHED] Total Pops: {sim.balloon_pop_count}, Total Simulated Time: {step * dt:.1f}s")
    print("=" * 65)
    assert sim.balloon_pop_count >= target_pops, f"Expected at least {target_pops} pops, got {sim.balloon_pop_count}"


if __name__ == "__main__":
    run_guidance_test()
