#!/bin/bash
# Test ARP/neighbor table integrity monitoring (defensive)
#
# This script runs the ARP monitor on the current (Linux) machine and watches:
#   - default gateway IP (recommended)
#   - optional extra IPs you provide interactively
#
# It does NOT perform any ARP spoofing or traffic manipulation.

set -e

echo "=========================================="
echo "Defense Test: ARP Integrity Monitoring"
echo "=========================================="
echo ""

if ! command -v ip >/dev/null 2>&1; then
  echo "❌ This test requires Linux 'ip' command (iproute2)."
  echo "   Run inside your Ubuntu VM."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFENSE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RESULTS_DIR="$DEFENSE_DIR/results"
mkdir -p "$RESULTS_DIR"

BASELINE_FILE="$RESULTS_DIR/arp_baseline_$(date +%Y%m%d_%H%M%S).json"
LOG_FILE="$RESULTS_DIR/arp_monitor_$(date +%Y%m%d_%H%M%S).log"

DEFAULT_ROUTE=$(ip route show default | head -1 || true)
DEFAULT_GW=$(echo "$DEFAULT_ROUTE" | awk '/default/ {for(i=1;i<=NF;i++) if ($i=="via") print $(i+1)}')
DEFAULT_DEV=$(echo "$DEFAULT_ROUTE" | awk '/default/ {for(i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}')

echo "Detected default route:"
echo "  Route: ${DEFAULT_ROUTE:-none}"
echo "  Gateway: ${DEFAULT_GW:-unknown}"
echo "  Interface: ${DEFAULT_DEV:-unknown}"
echo ""

read -p "Extra IPs to watch (space-separated, optional): " EXTRA_IPS
echo ""

echo "Starting ARP integrity monitor..."
echo "  Baseline: $BASELINE_FILE"
echo "  Log:      $LOG_FILE"
echo ""
echo "Press Ctrl+C to stop monitoring."
echo ""

python3 "$SCRIPT_DIR/monitor_arp.py" \
  --interface "${DEFAULT_DEV:-}" \
  --watch-gateway \
  --watch-ip $EXTRA_IPS \
  --interval 2 \
  --baseline-out "$BASELINE_FILE" \
  --log-file "$LOG_FILE" \
  --print-collisions

