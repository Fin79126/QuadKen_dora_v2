#!/usr/bin/env bash
# Launch Distributed Dora Coordinator and PC Daemon (Git Bash / Linux / macOS)
set -e

echo "=================================================="
echo "   Starting Dora Coordinator & Daemon on PC      "
echo "=================================================="

# 1. Bring up coordinator and local daemon
echo "[1/3] Bringing up Dora Coordinator..."
uv run dora up

# 2. Check cluster status
echo "[2/3] Checking cluster status..."
sleep 2
uv run dora status

echo ""
echo "[3/3] Ready to deploy dataflow."
echo "Once Raspberry Pi daemon has connected, deploy the dataflow with:"
echo "   uv run dora start dataflow.yml --name quadken"
echo ""
echo "To inspect traces:"
echo "   uv run dora trace list"
echo ""
echo "To stop dataflow and coordinator:"
echo "   uv run dora stop quadken"
echo "   uv run dora down"
