#!/usr/bin/env bash
# Network Layer MITM Delay Attack using Linux Traffic Control (tc)
set -euo pipefail

DELAY_MS="${1:-200}"
JITTER_MS="${2:-50}"

echo "=========================================="
echo "MITM Delay Attack: Adding ${DELAY_MS}ms delay to eth0"
echo "=========================================="

# Ensure we have clean state
sudo tc qdisc del dev eth0 root 2>/dev/null || true

# Add delay using netem (Network Emulator)
# This will affect ALL outgoing traffic from this container.
# In bridge mode, since the attacker is on the same subnet, 
# it can affect the communication if we route through it, 
# or we can simply demonstrate how network congestion affects secure nodes.
sudo tc qdisc add dev eth0 root netem delay "${DELAY_MS}ms" "${JITTER_MS}ms"

echo "Delay applied. Use 'sudo tc qdisc del dev eth0 root' to stop."
echo "Press Ctrl+C to exit this script (delay will remain until cleaned up)."

# Keep the script running to keep the pane active
while true; do
    sleep 1
done
