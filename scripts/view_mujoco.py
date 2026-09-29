"""
Interactive MuJoCo 3D Viewer for QuadKen AUV.
Run this script to open the native MuJoCo GUI, manipulate legs, thrusters, and ballast in real-time.

Usage:
  uv run python scripts/view_mujoco.py
"""

import time
import mujoco
import mujoco.viewer

MODEL_PATH = "assets/quadken.xml"


def main():
    print(f"Loading MuJoCo model from: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    print("Launching MuJoCo Native Viewer...")
    print("Controls:")
    print("  - Space: Pause / Resume simulation")
    print("  - Backspace: Reset simulation")
    print("  - Right Click + Drag: Rotate 3D Camera")
    print("  - Scroll Wheel: Zoom")
    print("  - Ctrl + Right Click: Pan Camera")
    print("  - Double Left Click on geom + Ctrl + Right Click: Apply 3D force/drag to robot")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            t = step_start - start_time

            # Example: Gently oscillate legs and thrust for demonstration
            # Actuator 0, 1: BLDC Thrusters
            # Actuator 2..5: Legs (Top, Right, Bottom, Left)
            # Actuator 6..9: Head Ballast Servos
            
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
