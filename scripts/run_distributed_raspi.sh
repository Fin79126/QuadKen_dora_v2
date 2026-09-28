#!/usr/bin/env bash
# Run on Raspberry Pi to join Dora coordinator on PC
# Usage: ./scripts/run_distributed_raspi.sh <PC_COORDINATOR_IP>

if [ -z "$1" ]; then
    echo "Usage: $0 <PC_COORDINATOR_IP>"
    echo "Example: $0 192.168.1.50"
    exit 1
fi

PC_IP="$1"

echo "=================================================="
echo "   Connecting Raspberry Pi to Dora Coordinator    "
echo "   Target PC IP: $PC_IP                           "
echo "=================================================="

# Ensure environment is synced
uv sync

# Launch Dora Daemon for machine-id: raspi
uv run dora daemon --coordinator-addr "$PC_IP" --machine-id raspi
