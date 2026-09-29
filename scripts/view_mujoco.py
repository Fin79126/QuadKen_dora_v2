"""
Interactive MuJoCo 3D Viewer for QuadKen AUV.

Controls:
  - Space: Pause / Resume simulation
  - Backspace: Reset simulation
  - 'A' key (in terminal) or argument: Toggle automatic demo animation ON/OFF
  - Right Click + Drag: Rotate 3D Camera
  - Scroll Wheel: Zoom
  - Ctrl + Right Click: Pan Camera

Usage:
  # Manual control mode (Use GUI sliders on the right panel to move legs/thrusters):
  uv run python scripts/view_mujoco.py

  # Automatic animation demo mode:
  uv run python scripts/view_mujoco.py --demo
"""

import sys
import time
import math
import argparse
import numpy as np
import mujoco
import mujoco.viewer

MODEL_PATH = "assets/quadken.xml"


def apply_membrane_hydrodynamics(model, data, auv_body_id):
    """Apply membrane drag and normal steering forces to auv root body."""
    rot_mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rot_mat, data.qpos[3:7])
    rot_mat = rot_mat.reshape((3, 3))

    world_lin_vel = data.qvel[0:3]
    body_lin_vel = rot_mat.T @ world_lin_vel
    u_forward = max(0.0, body_lin_vel[0])

    if u_forward <= 0.01:
        data.xfrc_applied[auv_body_id, :] = 0.0
        return

    water_density = 1000.0
    membrane_area_max = 0.08
    drag_coeff = 1.2
    lift_coeff = 1.0
    x_com = 0.28
    z_com = -0.02
    leg_length = 0.30
    hull_radius = 0.12

    q_dynamic = 0.5 * water_density * (u_forward ** 2)
    total_force = np.zeros(3)
    total_torque = np.zeros(3)

    # Actuators 2..5 correspond to legs [Top, Right, Bottom, Left]
    for i in range(4):
        angle_deg = max(0.0, min(90.0, data.ctrl[2 + i]))
        if angle_deg < 0.5:
            continue

        theta_rad = math.radians(angle_deg)
        sin_th = math.sin(theta_rad)
        cos_th = math.cos(theta_rad)

        area = membrane_area_max * sin_th
        f_drag = q_dynamic * drag_coeff * area
        f_normal = q_dynamic * lift_coeff * area * cos_th

        x_cp = -0.5 * leg_length * cos_th
        r_cp = hull_radius + 0.5 * leg_length * sin_th
        delta_x = x_cp - x_com

        if i == 0:  # Top
            r_vec = np.array([delta_x, 0.0, r_cp - z_com])
            f_vec = np.array([-f_drag, 0.0, -f_normal])
        elif i == 1:  # Right
            r_vec = np.array([delta_x, -r_cp, 0.0 - z_com])
            f_vec = np.array([-f_drag, +f_normal, 0.0])
        elif i == 2:  # Bottom
            r_vec = np.array([delta_x, 0.0, -r_cp - z_com])
            f_vec = np.array([-f_drag, 0.0, +f_normal])
        elif i == 3:  # Left
            r_vec = np.array([delta_x, +r_cp, 0.0 - z_com])
            f_vec = np.array([-f_drag, -f_normal, 0.0])

        torque_vec = np.cross(r_vec, f_vec)
        total_force += f_vec
        total_torque += torque_vec

    data.xfrc_applied[auv_body_id, 0:3] = rot_mat @ total_force
    data.xfrc_applied[auv_body_id, 3:6] = rot_mat @ total_torque


def main():
    parser = argparse.ArgumentParser(description="QuadKen AUV MuJoCo Viewer")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Enable automatic opening/closing demo animation (default: False for manual slider control)",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("      QuadKen AUV - MuJoCo Interactive 3D Viewer")
    print("=" * 60)
    print(f"Loading model: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    auv_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "auv")

    auto_animate = args.demo
    mode_str = "AUTO DEMO ANIMATION" if auto_animate else "MANUAL CONTROL (Use GUI Sliders)"
    print(f"Current Mode: {mode_str}")
    print("\nTips:")
    print("  - [Spacebar]: Pause / Resume physics")
    print("  - To control actuators manually, open the right 'Control' panel in the viewer window.")
    print("  - Units note: Control sliders are directly in DEGREES [0 to 90]!")
    if not auto_animate:
        print("  - Run with '--demo' to see automated leg deployment.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            t = step_start - start_time

            if auto_animate:
                # Oscillate legs 0 to 60 degrees directly
                cycle_angle_deg = 30.0 + 30.0 * math.sin(t * 1.5)
                for i in range(2, 6):
                    data.ctrl[i] = cycle_angle_deg
                # Forward thrust (5 N)
                data.ctrl[0] = 5.0
                data.ctrl[1] = 5.0

            # Apply membrane hydrodynamics (drag & rudder steering forces)
            apply_membrane_hydrodynamics(model, data, auv_body_id)

            # Step physics
            mujoco.mj_step(model, data)

            # Sync viewer state
            viewer.sync()

            # Maintain real-time step rate
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
