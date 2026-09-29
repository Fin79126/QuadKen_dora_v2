param(
    [switch]$InstantRespawn,
    [switch]$StaticBalloons
)

# Run QuadKen locally on Windows PC with MuJoCo Physics Simulation
# Includes: Virtual BNO055, Virtual Underwater Camera, Drag Steering Kinematics, Rerun 3D Visualizer
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "   Starting QuadKen MuJoCo SITL (dora-rs)         " -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Features active:" -ForegroundColor Yellow
Write-Host "  - 3D MuJoCo Physics Engine with Hydrodynamics" -ForegroundColor Gray
Write-Host "  - Virtual BNO055 IMU (Roll/Pitch/Yaw, Gyro, Accel, Depth)" -ForegroundColor Gray
Write-Host "  - Virtual Underwater Forward Camera with Target HUD" -ForegroundColor Gray
Write-Host "  - Autonomous AI Guidance & Strike Controller (Step 2 GNC)" -ForegroundColor Gray
Write-Host "  - Closed-loop Drag-Steering & Thrust Control" -ForegroundColor Gray
Write-Host "  - Rerun 3D Telemetry Visualizer" -ForegroundColor Gray

if ($InstantRespawn) {
    $env:BALLOON_RESPAWN_MODE = "instant"
    Write-Host "  - Balloon Mode: Instant Random Respawn (Endless)" -ForegroundColor Green
} else {
    $env:BALLOON_RESPAWN_MODE = "batch"
    Write-Host "  - Balloon Mode: Batch Random Respawn (10 per round)" -ForegroundColor Gray
}

if ($StaticBalloons) {
    $env:BALLOON_RANDOM_SPAWN = "0"
    Write-Host "  - Balloon Layout: Static XML Coordinates" -ForegroundColor Yellow
} else {
    $env:BALLOON_RANDOM_SPAWN = "1"
    Write-Host "  - Balloon Layout: Randomized across Pool Arena" -ForegroundColor Green
}
Write-Host ""

# Run Dora Dataflow with MuJoCo
uv run dora run dataflow_mujoco.yml --uv
