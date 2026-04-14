#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SESSION="${SESSION:-tb3_network_mitm_demo}"

usage() {
  cat <<EOF
Usage:
  $0

Environment options:
  SESSION=name   tmux session name, default: tb3_network_mitm_demo
  SKIP_CLEANUP=1 skip automatic cleanup before starting

This demo starts the isolated Docker bridge MITM lab and opens three panes:
  controller: 172.28.0.10
  robot:      172.28.0.20
  attacker:   172.28.0.50
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is not installed. Install it with: sudo apt install -y tmux" >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "ERROR: tmux session '$SESSION' already exists." >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  echo "Or stop it with: tmux kill-session -t $SESSION" >&2
  exit 1
fi

if [[ "${SKIP_CLEANUP:-0}" != "1" ]]; then
  "$SCRIPT_DIR/cleanup_all_experiments.sh"
fi

role_banner() {
  local title="$1"
  local role="$2"
  cat <<EOF
clear
cd "$PROJECT_ROOT"
printf '\\033[1;33m%s\\033[0m\\n' '============================================================'
printf '\\033[1;33m%s\\033[0m\\n' '$title'
printf '\\033[1;33m%s\\033[0m\\n' '============================================================'
printf 'Role: %s\\n' '$role'
printf 'Lab : Docker bridge network 172.28.0.0/24\\n'
printf '\\n'
EOF
}

controller_cmd() {
  cat <<'EOF'
sudo ./scripts/wsl_docker/mitm_up.sh
echo
echo 'Controller endpoint:'
sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; echo; echo 'ARP before:'; ip neigh"
echo
echo 'Ping robot endpoint to populate ARP cache:'
sudo ./scripts/wsl_docker/mitm_exec.sh controller "ping -c 3 172.28.0.20 || true; echo; echo 'ARP after:'; ip neigh"
echo
echo 'Optional ROS2 baseline publisher command:'
echo "sudo ./scripts/wsl_docker/mitm_exec.sh controller 'source /opt/ros/jazzy/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}, angular: {z: 0.0}}\"'"
echo
read -r -p 'Press Enter to keep controller pane open... ' _
EOF
}

robot_cmd() {
  cat <<'EOF'
sleep 5
echo 'Robot endpoint:'
sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; echo; echo 'ARP table:'; ip neigh"
echo
echo 'Optional ROS2 baseline subscriber command:'
echo "sudo ./scripts/wsl_docker/mitm_exec.sh robot 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /cmd_vel'"
echo
read -r -p 'Press Enter to keep robot pane open... ' _
EOF
}

attacker_cmd() {
  cat <<'EOF'
sleep 7
echo 'Attacker endpoint with NET_ADMIN/NET_RAW:'
sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; echo; echo 'Capabilities relevant to MITM:'; command -v tcpdump; echo; echo 'ARP table:'; ip neigh"
echo
echo 'Safe evidence capture command:'
echo "sudo ./experiments/03_network_mitm/collect_evidence.sh"
echo
echo 'ARP poisoning is intentionally not auto-started by this demo.'
echo 'Dry-run command:'
echo "sudo ./scripts/wsl_docker/mitm_arp_poison.sh --duration 6"
echo
echo 'Execute command for this isolated Docker bridge lab only:'
echo "sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20"
echo
read -r -p 'Press Enter to keep attacker pane open... ' _
EOF
}

tmux new-session -d -s "$SESSION" -n "network-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "MITM_CONTROLLER_172.28.0.10"
tmux select-pane -t "$SESSION:0.1" -T "MITM_ROBOT_172.28.0.20"
tmux select-pane -t "$SESSION:0.2" -T "MITM_ATTACKER_172.28.0.50"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

tmux send-keys -t "$SESSION:0.0" "$(role_banner 'ROLE 1: CONTROLLER ENDPOINT' 'legitimate sender on Docker bridge') $(controller_cmd)" C-m
tmux send-keys -t "$SESSION:0.1" "$(role_banner 'ROLE 2: ROBOT ENDPOINT' 'target endpoint on Docker bridge') $(robot_cmd)" C-m
tmux send-keys -t "$SESSION:0.2" "$(role_banner 'ROLE 3: ATTACKER / MITM' 'network-path attacker on Docker bridge') $(attacker_cmd)" C-m

cat <<EOF
Started tmux session: $SESSION

Attach:
  tmux attach -t $SESSION

Stop demo:
  tmux kill-session -t $SESSION

Stop MITM lab containers:
  sudo ./scripts/wsl_docker/mitm_down.sh
EOF

tmux attach -t "$SESSION"
