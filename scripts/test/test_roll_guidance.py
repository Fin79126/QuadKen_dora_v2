"""
Test script to inspect roll compensation behavior in AI guidance.
Checks whether roll angle is correctly accounted for when steering toward the balloon.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim
from nodes.pc.compute import UnderwaterDynamics
from nodes.pc.ai_guidance import AIGuidanceController
import mujoco

def test_roll_steering(init_roll_deg: float = 30.0):
    sim = QuadKenMuJoCoSim()
    dynamics = UnderwaterDynamics()
    guidance = AIGuidanceController()

    roll_rad = np.radians(init_roll_deg)
    cr, sr = np.cos(roll_rad / 2.0), np.sin(roll_rad / 2.0)
    sim.data.qpos[3:7] = [cr, sr, 0.0, 0.0]
    mujoco.mj_forward(sim.model, sim.data)

    print(f"=== Testing with Initial Roll = {init_roll_deg} deg ===")
    dt = 0.02
    for step in range(500):
        t_info = sim.get_target_relative_info()
        bno = sim.get_bno_payload(step)
        cmd, status = guidance.update(t_info, bno, step * dt)
        leg_angles, bldc, ballast = dynamics.compute_actuators(
            dt,
            cmd["throttle"],
            cmd["steer_yaw"],
            cmd["steer_pitch"],
            cmd["steer_roll"],
            cmd["ballast"],
            cmd["brake"],
            bno["roll"],
            bno["pitch"],
            cmd["stick_right_x"],
            cmd["stick_right_y"],
        )
        sim.set_actuator_commands({"esp1": {"motors": bldc, "servos": leg_angles}, "esp2": {"servos": ballast}})
        sim.step()
        if step % 50 == 0:
            print(
                f"t={step*dt:4.1f}s | Roll:{bno['roll']:+5.1f} | Dist:{t_info['distance_m']:4.2f}m | "
                f"Az:{t_info['azimuth_deg']:+5.1f} | El:{t_info['elevation_deg']:+5.1f} | "
                f"Cmd:[x={cmd['stick_right_x']:+4.2f}, y={cmd['stick_right_y']:+4.2f}] | "
                f"Legs(T,R,B,L):{[int(x) for x in leg_angles]} | Pops:{sim.balloon_pop_count}"
            )
        if sim.balloon_pop_count > 0:
            print(f">>> POPPED BALLOON AT t={step*dt:.2f}s with Roll={bno['roll']:.1f} deg! <<<")
            break

if __name__ == "__main__":
    test_roll_steering(30.0)
    test_roll_steering(60.0)
    test_roll_steering(-45.0)
    test_roll_steering(180.0)

