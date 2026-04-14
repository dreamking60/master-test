#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  sudo ./scripts/wsl_docker/mitm_arp_poison.sh [options]

Common options:
  --duration SECONDS       How long to run the loop
  --interval SECONDS       Delay between ARP reply pairs
  --execute                Actually send ARP replies; default is dry-run
  --restore                Send restoration ARP replies at exit

Examples:
  sudo ./scripts/wsl_docker/mitm_arp_poison.sh --duration 6
  sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20

This wrapper runs the constrained helper inside tb3-mitm-attacker.
EOF
  exit 0
fi

"$SCRIPT_DIR/mitm_exec.sh" attacker \
  "python3 /workspace/project/experiments/03_network_mitm/arp_poison_lab.py $*"
