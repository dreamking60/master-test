#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$PROJECT_ROOT/logs/experiments/03_network_mitm"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$OUT_DIR/evidence_$STAMP.log"

mkdir -p "$OUT_DIR"

run_section() {
  local title="$1"
  local command="$2"
  {
    echo
    echo "## $title"
    echo "\$ $command"
    bash -lc "$command" 2>&1 || true
  } >> "$OUT"
}

{
  echo "# Network MITM bridge-lab evidence"
  echo "Timestamp: $STAMP"
  echo "Project: $PROJECT_ROOT"
} > "$OUT"

run_section "Docker containers" "sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}' | grep -E 'NAMES|tb3-mitm' || true"
run_section "Docker network inspect" "sudo docker network inspect wsl_docker_mitm_lab"
run_section "Controller network state" "sudo ./scripts/wsl_docker/mitm_exec.sh controller 'hostname; ip -br addr; ip route; ip neigh'"
run_section "Robot network state" "sudo ./scripts/wsl_docker/mitm_exec.sh robot 'hostname; ip -br addr; ip route; ip neigh'"
run_section "Attacker network state" "sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'hostname; ip -br addr; ip route; ip neigh'"
run_section "Controller to robot ping" "sudo ./scripts/wsl_docker/mitm_exec.sh controller 'ping -c 3 172.28.0.20'"
run_section "Attacker sees controller/robot" "sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'ping -c 2 172.28.0.10; ping -c 2 172.28.0.20'"
run_section "Recent ARP helper logs" "find '$PROJECT_ROOT/logs/experiments/03_network_mitm' -name 'arp_poison_*.jsonl' -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -3 | cut -d' ' -f2- | xargs -r -I{} sh -c 'echo --- {}; tail -n 40 {}'"

echo "Evidence written to: $OUT"
