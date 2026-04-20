#!/usr/bin/env bash
# SROS2 MITM Delay Attack Demo (Network Layer)
#
# This script uses Docker Bridge mode to demonstrate how network-level
# delay impacts a secure ROS2 control loop.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SESSION="${SESSION:-sros2_mitm_demo}"
ATTACK_DELAY="${ATTACK_DELAY:-0}"

if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is not installed." >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

if [[ "${SKIP_CLEANUP:-0}" != "1" ]]; then
  "$SCRIPT_DIR/cleanup_all_experiments.sh"
fi

# Ensure keys are generated
if [ ! -d "$PROJECT_ROOT/deployment/wsl_docker/keys" ]; then
    echo "SROS2 keys not found. Generating..."
    sudo "$PROJECT_ROOT/scripts/wsl_docker/init_sros2_docker.sh"
fi

tmux new-session -d -s "$SESSION" -n "sros2-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "ROBOT_SIM"
tmux select-pane -t "$SESSION:0.1" -T "SECURE_CONTROLLER"
tmux select-pane -t "$SESSION:0.2" -T "MITM_ATTACKER"
tmux select-pane -t "$SESSION:0.3" -T "NETWORK_MONITOR"

tmux send-keys -t "$SESSION:0.0" "ATTACK_DELAY='$ATTACK_DELAY' ./scripts/demo/sros2_mitm_pane.sh robot" C-m
tmux send-keys -t "$SESSION:0.1" "ATTACK_DELAY='$ATTACK_DELAY' ./scripts/demo/sros2_mitm_pane.sh controller" C-m
tmux send-keys -t "$SESSION:0.2" "ATTACK_DELAY='$ATTACK_DELAY' ./scripts/demo/sros2_mitm_pane.sh attacker" C-m
tmux send-keys -t "$SESSION:0.3" "ATTACK_DELAY='$ATTACK_DELAY' ./scripts/demo/sros2_mitm_pane.sh monitor" C-m

tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null

echo "Started MITM Delay Demo: $SESSION"
tmux attach-session -t "$SESSION"
