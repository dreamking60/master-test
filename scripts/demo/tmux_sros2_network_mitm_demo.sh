#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SESSION="${SESSION:-tb3_sros2_network_mitm_demo}"

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

if [ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/mitm_controller/permissions.p7s" ] || \
   [ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/mitm_robot/permissions.p7s" ]; then
  echo "ERROR: SROS2 MITM permissions not found." >&2
  echo "Run first:" >&2
  echo "  sudo ./scripts/wsl_docker/init_sros2_docker.sh" >&2
  exit 1
fi

tmux new-session -d -s "$SESSION" -n "sros2-network-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "SROS2_CONTROLLER"
tmux select-pane -t "$SESSION:0.1" -T "SROS2_ROBOT"
tmux select-pane -t "$SESSION:0.2" -T "MITM_ATTACKER"
tmux select-pane -t "$SESSION:0.3" -T "MONITOR"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

COMMON_ENV="ATTACK_DELAY_MS='${ATTACK_DELAY_MS:-500}' ATTACK_LOSS_PERCENT='${ATTACK_LOSS_PERCENT:-5}' ATTACK_DURATION='${ATTACK_DURATION:-45}'"
tmux send-keys -t "$SESSION:0.0" "$COMMON_ENV ./scripts/demo/sros2_network_mitm_pane.sh controller" C-m
tmux send-keys -t "$SESSION:0.1" "$COMMON_ENV ./scripts/demo/sros2_network_mitm_pane.sh robot" C-m
tmux send-keys -t "$SESSION:0.2" "$COMMON_ENV ./scripts/demo/sros2_network_mitm_pane.sh attacker" C-m
tmux send-keys -t "$SESSION:0.3" "$COMMON_ENV ./scripts/demo/sros2_network_mitm_pane.sh monitor" C-m

cat <<MSG
Started SROS2 Network MITM demo: $SESSION

Attach:
  tmux attach -t $SESSION

Stop:
  tmux kill-session -t $SESSION
MSG

tmux attach -t "$SESSION"
