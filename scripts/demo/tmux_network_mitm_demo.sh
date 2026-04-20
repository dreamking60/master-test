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

tmux new-session -d -s "$SESSION" -n "network-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "MITM_CONTROLLER_172.28.0.10"
tmux select-pane -t "$SESSION:0.1" -T "MITM_ROBOT_172.28.0.20"
tmux select-pane -t "$SESSION:0.2" -T "MITM_ATTACKER_172.28.0.50"
tmux select-pane -t "$SESSION:0.3" -T "MITM_MONITOR"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

tmux send-keys -t "$SESSION:0.0" "./scripts/demo/network_mitm_pane.sh controller" C-m
tmux send-keys -t "$SESSION:0.1" "./scripts/demo/network_mitm_pane.sh robot" C-m
tmux send-keys -t "$SESSION:0.2" "./scripts/demo/network_mitm_pane.sh attacker" C-m
tmux send-keys -t "$SESSION:0.3" "./scripts/demo/network_mitm_pane.sh monitor" C-m

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
