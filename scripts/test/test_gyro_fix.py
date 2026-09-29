import sys
import os
import mujoco
import numpy as np

sys.path.insert(0, r"c:\Users\tacky\MyApp\QuadKen\QuadKen_dora_v2")

from nodes.simulation.mujoco_node import QuadKenMuJoCoSim
from nodes.pc.compute import UnderwaterDynamics
from nodes.pc.ai_guidance import AIGuidanceController

def test_gyro_options(init_roll_deg, mode="world_gyro"):
    sim = QuadKenMuJoCoSim()
    dynamics = UnderwaterDynamics()
    guidance = AIGuidanceController()

    roll_rad = np.radians(init_roll_deg)
    cr, sr = np.cos(roll_rad / 2.0), np.sin(roll_rad / 2.0)
    sim.data.qpos[3:7] = [cr, sr, 0.0, 0.0]
    mujoco.mj_forward(sim.model, sim.data)

    dt = 0.02
    prev_leg_angles = [0.0]*4
    leg_oscillation_energy = 0.0
    yaw_rate_oscillation = 0.0
    pitch_rate_oscillation = 0.0

    for step in range(300):
        t_info = sim.get_target_relative_info()
        bno = sim.get_bno_payload(step)

        # Mode calculation
        roll_deg = float(bno.get("roll", 0.0))
        pitch_deg = float(bno.get("pitch", 0.0))
        r_rad = np.radians(roll_deg)
        p_rad = np.radians(pitch_deg)

        azimuth_deg = float(t_info.get("azimuth_deg", 0.0))
        elevation_deg = float(t_info.get("elevation_deg", 0.0))
        az_rad = np.radians(azimuth_deg)
        el_rad = np.radians(elevation_deg)

        gyro = bno.get("gyro", [0.0, 0.0, 0.0]) # [gx, gy, gz] deg/s

        # De-roll
        xb = np.cos(el_rad) * np.cos(az_rad)
        yb = -np.cos(el_rad) * np.sin(az_rad)
        zb = np.sin(el_rad)

        y_level = np.cos(r_rad) * yb - np.sin(r_rad) * zb
        z_level = np.sin(r_rad) * yb + np.cos(r_rad) * zb

        azimuth_level = np.degrees(np.arctan2(-y_level, max(1e-4, xb)))
        elevation_level = np.degrees(np.arctan2(z_level, np.hypot(xb, y_level)))

        d_az = (azimuth_level - guidance.prev_azimuth_err) / dt
        d_el = (elevation_level - guidance.prev_elevation_err) / dt
        guidance.prev_azimuth_err = azimuth_level
        guidance.prev_elevation_err = elevation_level

        if mode == "raw_gyro":
            # Original code
            yaw_cmd = guidance.kp_yaw * azimuth_level + guidance.kd_yaw * d_az + guidance.k_gyro_yaw * gyro[2]
        elif mode == "zero_gyro":
            # Just PD
            yaw_cmd = guidance.kp_yaw * azimuth_level + guidance.kd_yaw * d_az
        elif mode == "world_gyro":
            # Correct Earth-level yaw rate projection:
            # omega_z_world = -p*sin(pitch) + q*sin(roll)*cos(pitch) + r*cos(roll)*cos(pitch)
            omega_z_world = (
                -gyro[0] * np.sin(p_rad)
                + gyro[1] * np.sin(r_rad) * np.cos(p_rad)
                + gyro[2] * np.cos(r_rad) * np.cos(p_rad)
            )
            yaw_cmd = guidance.kp_yaw * azimuth_level + guidance.kd_yaw * d_az + guidance.k_gyro_yaw * omega_z_world

        pitch_cmd = guidance.kp_pitch * elevation_level + guidance.kd_pitch * d_el

        steer_yaw = float(np.clip(yaw_cmd, -1.0, 1.0))
        steer_pitch = float(np.clip(pitch_cmd, -1.0, 1.0))

        cmd = {
            "throttle": 0.9,
            "steer_yaw": steer_yaw,
            "steer_pitch": steer_pitch,
            "steer_roll": 0.0,
            "stick_right_x": steer_yaw,
            "stick_right_y": steer_pitch,
            "ballast": 0.0,
            "brake": False,
        }

        leg_angles, bldc, ballast = dynamics.compute_actuators(
            dt, cmd["throttle"], cmd["steer_yaw"], cmd["steer_pitch"], cmd["steer_roll"],
            cmd["ballast"], cmd["brake"], bno["roll"], bno["pitch"],
            cmd["stick_right_x"], cmd["stick_right_y"]
        )
        sim.set_actuator_commands({"esp1": {"motors": bldc, "servos": leg_angles}, "esp2": {"servos": ballast}})
        sim.step()

        d_legs = [abs(leg_angles[i] - prev_leg_angles[i]) / dt for i in range(4)]
        leg_oscillation_energy += sum(d_legs) * dt
        prev_leg_angles = list(leg_angles)

        yaw_rate_oscillation += abs(bno["gyro"][2]) * dt
        pitch_rate_oscillation += abs(bno["gyro"][1]) * dt

        if sim.balloon_pop_count > 0:
            break

    return leg_oscillation_energy, yaw_rate_oscillation, pitch_rate_oscillation, sim.balloon_pop_count, step * dt

rolls = [0.0, 60.0, 120.0, 150.0, 180.0]
print(f"{'Roll':>6s} | {'Mode':>10s} | {'LegChatter':>10s} | {'YawRate':>8s} | {'PitchRate':>9s} | {'Popped':>6s} | {'Time':>6s}")
print("-" * 70)
for r in rolls:
    for m in ["raw_gyro", "zero_gyro", "world_gyro"]:
        leg, yaw, pitch, pop, t = test_gyro_options(r, m)
        print(f"{r:+6.1f} | {m:>10s} | {leg:10.1f} | {yaw:8.1f} | {pitch:9.1f} | {pop:6d} | {t:5.2f}s")
