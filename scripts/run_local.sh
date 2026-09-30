#!/usr/bin/env bash
# Run QuadKen locally on PC (Git Bash / Linux / macOS)
set -e

echo "=================================================="
echo "   Starting QuadKen Dora v2 (Local PC Mode)      "
echo "=================================================="

# Function to clean up background processes in Git Bash & Linux
cleanup() {
    echo ""
    echo "Stopping ESP Simulator..."
    if [ -n "$SIM_PID" ]; then
        kill -9 "$SIM_PID" 2>/dev/null || true
        # Also clean up child processes on Windows if running in Git Bash
        if command -v taskkill &> /dev/null; then
            taskkill //F //T //PID "$SIM_PID" 2>/dev/null || true
        fi
    fi
    echo "Finished."
}
trap cleanup EXIT INT TERM

# 1. Start ESP Simulator in background
echo "[1/2] Starting ESP1 & ESP2 Simulator..."
uv run python esp/esp_simulator.py &
SIM_PID=$!

sleep 1

# 2. Launch Dora Dataflow
echo "[2/2] Launching Dora Dataflow..."
uv run dora run dataflow_local.yml --uv --log-filter "camera=error"
