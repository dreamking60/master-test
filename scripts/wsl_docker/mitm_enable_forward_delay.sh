#!/usr/bin/env bash
set -euo pipefail

DELAY_MS="${1:-250}"
LOSS_PERCENT="${2:-0}"

echo "Enabling IPv4 forwarding in attacker container..."
if command -v sysctl >/dev/null 2>&1; then
  sysctl -w net.ipv4.ip_forward=1
else
  echo 1 > /proc/sys/net/ipv4/ip_forward
fi

echo "Applying netem delay on attacker eth0: delay=${DELAY_MS}ms loss=${LOSS_PERCENT}%"
tc qdisc del dev eth0 root 2>/dev/null || true
tc qdisc add dev eth0 root netem delay "${DELAY_MS}ms" loss "${LOSS_PERCENT}%"

echo
echo "Current forwarding state:"
cat /proc/sys/net/ipv4/ip_forward
echo
echo "Current qdisc:"
tc qdisc show dev eth0
