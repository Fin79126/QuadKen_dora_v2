"""
QuadKen Step 4 Verification: Perception-in-the-Loop Full Autonomous Destruction Benchmark.

Validates:
  1. Complete perceptual autonomy: Zero reliance on simulation ground-truth target coordinates.
     All guidance errors (azimuth, elevation, distance) are calculated exclusively from OpenCV camera perception.
  2. Search & Acquisition: Automatic wide-angle sweep and depth-holding patrol when target is out of view.
  3. Collision mitigation & Post-Pop recovery: Immediate deceleration, nose-up pitch trim, and rapid next-target search.
  4. Multi-balloon sequential destruction benchmark across varied 3D positions in the pool arena.
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


def run_benchmark(target_pops: int = 3, max_sim_seconds: float = 70.0, random_spawn: bool = False):
    spawn_desc = "Random Placement" if random_spawn else "Fixed Benchmark Layout"
    print("\n" + "=" * 72)
    print(f" [BENCHMARK] Step 4 Full Autonomous Destruction Test ({spawn_desc})")
    print(f" Target Goal: {target_pops} Balloon Destructions | Max Time: {max_sim_seconds:.1f}s")
    print("=" * 72)

    sim = QuadKenMuJoCoSim(random_spawn=random_spawn)
    detector = BalloonDetector()
    guidance = AIGuidanceController()
    dynamics = UnderwaterDynamics()

    dt = 0.02  # 50 Hz physics step
    total_steps = int(max_sim_seconds / dt)

    pop_history = []
    depths = []
    modes_count = {}

    for step in range(total_steps):
        current_time = step * dt

        # 1. Physics & Telemetry
        bno_data = sim.get_bno_payload(step)
        gt_info = sim.get_target_relative_info()
        depths.append(bno_data["depth_m"])

        # 2. Camera Capture
        raw_bgr = sim.render_front_camera(return_raw_bgr=True)

        # 3. Vision Perception (OpenCV)
        vision_info, annotated_bgr = detector.detect(raw_bgr, bno_data)

        # 4. Pop Event Monitoring
        pop_cnt = sim.balloon_pop_count
        if pop_cnt > len(pop_history):
            pop_history.append({
                "pop_index": pop_cnt,
                "sim_time": current_time,
                "pos": [round(float(p), 2) for p in sim.data.qpos[0:3]],
            })
            print(f">>> [HIT #{pop_cnt}] at t={current_time:5.2f}s | Robot Pos: {pop_history[-1]['pos']} | Mode: {guidance.current_mode} <<<")
            if len(pop_history) >= target_pops:
                print(f"\n[BENCHMARK SUCCESS] Successfully achieved target {target_pops} pops in {current_time:.2f}s!")
                break

        # 5. GNC Guidance (Exclusively from vision_info + bno_data)
        cmd, status = guidance.update(vision_info, bno_data, current_time)
        mode = status["mode"]
        modes_count[mode] = modes_count.get(mode, 0) + 1

        # 6. Actuator Kinematics
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

        # 7. Apply to MuJoCo simulation
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
        sim.step()

        # Telemetry log every 1 second
        if step % 50 == 0:
            found_str = "LOCKED" if vision_info["target_found"] else "SEARCHING"
            p = sim.data.qpos[0:3]
            print(
                f"[t={current_time:5.1f}s] {found_str:9s} Mode:{status['mode']:10s} "
                f"Dist:{vision_info.get('distance_m', 0.0):4.2f}m "
                f"Az:{vision_info.get('azimuth_deg', 0.0):+5.1f} "
                f"Pitch:{bno_data['pitch']:+5.1f} Depth:{bno_data['depth_m']:4.2f}m "
                f"Thr:{status['throttle']:.2f} Pops:{pop_cnt}"
            )

    sim_duration = min(max_sim_seconds, (step + 1) * dt)
    print("\n" + "=" * 72)
    print(" [BENCHMARK RESULTS SUMMARY]")
    print("=" * 72)
    print(f" Total Balloons Popped: {len(pop_history)} / {target_pops}")
    print(f" Total Elapsed Time   : {sim_duration:.2f} s")
    if pop_history:
        for p in pop_history:
            print(f"   - Pop #{p['pop_index']}: at t={p['sim_time']:5.2f}s (Pos: {p['pos']})")
    
    mean_depth = float(np.mean(depths))
    min_depth = float(np.min(depths))
    max_depth = float(np.max(depths))
    print(f" Depth Statistics     : Mean={mean_depth:.2f}m (Min={min_depth:.2f}m, Max={max_depth:.2f}m)")
    print(f" Mode Distributions   : {modes_count}")
    print("=" * 72 + "\n")

    assert len(pop_history) >= target_pops, f"Benchmark failed: only {len(pop_history)} pops achieved out of {target_pops}"
    return pop_history


if __name__ == "__main__":
    # Test 1: Standard layout benchmark (3 balloons)
    print("\n>>> RUNNING TEST SUITE 1: FIXED BENCHMARK LAYOUT <<<")
    run_benchmark(target_pops=3, max_sim_seconds=45.0, random_spawn=False)

    # Test 2: Randomized arena benchmark (3 balloons with random 3D spawn across pool)
    print("\n>>> RUNNING TEST SUITE 2: RANDOM ARENA LAYOUT <<<")
    run_benchmark(target_pops=3, max_sim_seconds=90.0, random_spawn=True)

