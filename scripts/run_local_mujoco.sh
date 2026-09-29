#!/usr/bin/env bash
# Run QuadKen locally with MuJoCo Physics Simulation
set -e

echo "=================================================="
echo "   Starting QuadKen MuJoCo SITL (dora-rs)         "
echo "=================================================="
echo "Features active:"
echo "  - 3D MuJoCo Physics Engine with Hydrodynamics"
echo "  - Virtual BNO055 IMU (Roll/Pitch/Yaw, Gyro, Accel, Depth)"
echo "  - Virtual Underwater Forward Camera with Target HUD"
echo "  - Autonomous AI Guidance & Strike Controller (Step 2 GNC)"
echo "  - Closed-loop Drag-Steering & Thrust Control"
echo "  - Rerun 3D Telemetry Visualizer"
echo ""

export BALLOON_RANDOM_SPAWN="${BALLOON_RANDOM_SPAWN:-1}"
export BALLOON_RESPAWN_MODE="${BALLOON_RESPAWN_MODE:-batch}"

uv run dora run dataflow_mujoco.yml --uv
