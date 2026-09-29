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
import mujoco
import mujoco.viewer

MODEL_PATH = "assets/quadken.xml"


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

    auto_animate = args.demo
    mode_str = "AUTO DEMO ANIMATION" if auto_animate else "MANUAL CONTROL (Use GUI Sliders)"
    print(f"Current Mode: {mode_str}")
    print("\nTips:")
    print("  - [Spacebar]: Pause / Resume physics")
    print("  - To control actuators manually, open the right 'Control' panel in the viewer window.")
    print("  - Units note: 1.0 on slider = 1 radian (~57.3 deg). 1.57 = 90 deg.")
    if not auto_animate:
        print("  - Run with '--demo' to see automated leg deployment.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            t = step_start - start_time

            if auto_animate:
                # Oscillate legs 0 to 60 deg (0 to ~1.05 rad)
                cycle_angle_deg = 30.0 + 30.0 * math.sin(t * 1.5)
                cycle_angle_rad = math.radians(cycle_angle_deg)
                for i in range(2, 6):
                    data.ctrl[i] = cycle_angle_rad
                # Forward thrust
                data.ctrl[0] = 5.0
                data.ctrl[1] = 5.0

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
