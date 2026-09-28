#!/usr/bin/env bash
# Run QuadKen locally on Linux / macOS / WSL
set -e

echo "=================================================="
echo "   Starting QuadKen Dora v2 (Local PC Mode)      "
echo "=================================================="

# Start ESP Simulator in background
echo "[1/2] Starting ESP1 & ESP2 Simulator..."
uv run python esp/esp_simulator.py &
SIM_PID=$!

trap "echo 'Stopping ESP Simulator...'; kill -9 $SIM_PID 2>/dev/null || true" EXIT

sleep 1

# Launch Dora Dataflow
echo "[2/2] Launching Dora Dataflow..."
uv run dora run dataflow_local.yml --uv
