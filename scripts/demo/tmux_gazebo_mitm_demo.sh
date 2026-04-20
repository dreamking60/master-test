#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SESSION="${SESSION:-tb3_gazebo_mitm_demo}"

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

tmux new-session -d -s "$SESSION" -n "gazebo-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "WSL_GAZEBO"
tmux select-pane -t "$SESSION:0.1" -T "MITM_CONTROLLER"
tmux select-pane -t "$SESSION:0.2" -T "ROBOT_GATEWAY"
tmux select-pane -t "$SESSION:0.3" -T "MITM_ATTACKER"
tmux select-pane -t "$SESSION:0.4" -T "MONITOR"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

COMMON_ENV="ATTACK_DELAY_MS='${ATTACK_DELAY_MS:-500}' ATTACK_LOSS_PERCENT='${ATTACK_LOSS_PERCENT:-5}' ATTACK_DURATION='${ATTACK_DURATION:-45}' MITM_TAMPER_ENABLE='${MITM_TAMPER_ENABLE:-0}' MITM_TAMPER_ANGULAR_Z='${MITM_TAMPER_ANGULAR_Z:-1.2}'"

tmux send-keys -t "$SESSION:0.0" "$COMMON_ENV ./scripts/demo/gazebo_mitm_pane.sh gazebo" C-m
tmux send-keys -t "$SESSION:0.1" "$COMMON_ENV ./scripts/demo/gazebo_mitm_pane.sh controller" C-m
tmux send-keys -t "$SESSION:0.2" "$COMMON_ENV ./scripts/demo/gazebo_mitm_pane.sh robot" C-m
tmux send-keys -t "$SESSION:0.3" "$COMMON_ENV ./scripts/demo/gazebo_mitm_pane.sh attacker" C-m
tmux send-keys -t "$SESSION:0.4" "$COMMON_ENV ./scripts/demo/gazebo_mitm_pane.sh monitor" C-m

cat <<MSG
Started Gazebo MITM demo: $SESSION

Attach:
  tmux attach -t $SESSION

Stop:
  tmux kill-session -t $SESSION
MSG

tmux attach -t "$SESSION"
