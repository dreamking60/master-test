#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SESSION="${SESSION:-tb3_security_demo}"
MODE="${1:-open}"
ATTACK_DELAY="${ATTACK_DELAY:-0}"

usage() {
  cat <<EOF
Usage:
  $0 [open|secure]

Environment options:
  SESSION=name        tmux session name, default: tb3_security_demo
  ATTACK_DELAY=secs   attacker auto-start delay. Default 0 means wait for Enter.

Examples:
  $0 open
  ATTACK_DELAY=12 $0 open
  $0 secure
EOF
}

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  usage
  exit 0
fi

if [[ "$MODE" != "open" && "$MODE" != "secure" ]]; then
  echo "ERROR: mode must be 'open' or 'secure'." >&2
  usage >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "ERROR: tmux is not installed." >&2
  echo "Install it with: sudo apt install -y tmux" >&2
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "ERROR: tmux session '$SESSION' already exists." >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  echo "Or stop it with: tmux kill-session -t $SESSION" >&2
  exit 1
fi

banner_cmd() {
  local title="$1"
  local role="$2"
  local topic="$3"
  cat <<EOF
clear
cd "$PROJECT_ROOT"
$(tmux_env_exports)
printf '\\033[1;36m%s\\033[0m\\n' '============================================================'
printf '\\033[1;36m%s\\033[0m\\n' '$title'
printf '\\033[1;36m%s\\033[0m\\n' '============================================================'
printf 'Role : %s\\n' '$role'
printf 'Topic: %s\\n' '$topic'
printf 'Mode : %s\\n' '$MODE'
printf '\\n'
EOF
}

tmux_env_exports() {
  local name
  for name in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR PULSE_SERVER DBUS_SESSION_BUS_ADDRESS WSL_INTEROP WSL_DISTRO_NAME; do
    if [[ -n "${!name:-}" ]]; then
      printf 'export %s=%q\n' "$name" "${!name}"
    fi
  done
}

gazebo_pane_run_cmd() {
  cat <<EOF
touch test.log
echo 'Starting Gazebo in background; live logs will appear below.'
echo "DISPLAY=\${DISPLAY:-<empty>}"
echo "WAYLAND_DISPLAY=\${WAYLAND_DISPLAY:-<empty>}"
echo "XDG_RUNTIME_DIR=\${XDG_RUNTIME_DIR:-<empty>}"
echo 'If the GUI does not appear, these WSLg variables are the first thing to check.'
echo
($GAZEBO_RUN; echo "[GAZEBO_EXIT] command exited with status \$?") &
GAZEBO_PID=\$!
sleep 6
echo
echo '--- live test.log ---'
tail -n 120 -f test.log &
TAIL_PID=\$!
wait \$GAZEBO_PID
STATUS=\$?
kill \$TAIL_PID 2>/dev/null || true
wait \$TAIL_PID 2>/dev/null || true
echo
echo "Gazebo command finished with status \$STATUS. Press Enter to keep this pane open."
read -r _
EOF
}

if [[ "$MODE" == "open" ]]; then
  GAZEBO_RUN="./scripts/setup/run_wsl_gazebo.sh"
  CONTROLLER_RUN="sudo ./scripts/wsl_docker/start_controller_stack.sh"
  ATTACKER_RUN="sudo ./scripts/wsl_docker/run_attacker.sh"
  GAZEBO_ROLE="Robot/Gazebo role on WSL host, not Docker. Runs Gazebo, ros_gz_bridge, and cmd_vel_relay."
  CONTROLLER_ROLE="Controller machine in Docker. Publishes legitimate forward commands to /cmd_vel_in."
  ATTACKER_ROLE="Attacker machine in Docker. Publishes unauthorized turn commands to /cmd_vel_in."
else
  GAZEBO_RUN="./scripts/setup/run_wsl_gazebo_secure.sh"
  CONTROLLER_RUN="sudo ./scripts/wsl_docker/secure_start_controller_stack.sh"
  ATTACKER_RUN="sudo ./scripts/wsl_docker/run_attacker.sh"
  GAZEBO_ROLE="Secure Robot/Gazebo role on WSL host, not Docker. Uses SROS2 enclave /gazebo."
  CONTROLLER_ROLE="Secure controller machine in Docker. Uses SROS2 enclave /controller."
  ATTACKER_ROLE="Attacker machine in Docker. Uses enclave /attacker but lacks command-topic permission."
fi

if [[ "$ATTACK_DELAY" == "0" ]]; then
  ATTACK_GATE="read -r -p 'Press Enter to start attacker when you are ready for the demo... '"
else
  ATTACK_GATE="echo 'Attacker will start after ${ATTACK_DELAY}s...'; sleep '$ATTACK_DELAY'"
fi

tmux new-session -d -s "$SESSION" -n "three-machines" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "ROBOT_GAZEBO_WSL_HOST"
tmux select-pane -t "$SESSION:0.1" -T "CONTROLLER_DOCKER"
tmux select-pane -t "$SESSION:0.2" -T "ATTACKER_DOCKER"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

tmux send-keys -t "$SESSION:0.0" "$(banner_cmd 'ROLE 1: ROBOT / GAZEBO (WSL HOST)' "$GAZEBO_ROLE" '/cmd_vel via ros_gz_bridge') $(gazebo_pane_run_cmd)" C-m
tmux send-keys -t "$SESSION:0.1" "$(banner_cmd 'ROLE 2: CONTROLLER (DOCKER)' "$CONTROLLER_ROLE" '/cmd_vel_in') sleep 8; $CONTROLLER_RUN" C-m
tmux send-keys -t "$SESSION:0.2" "$(banner_cmd 'ROLE 3: ATTACKER (DOCKER)' "$ATTACKER_ROLE" '/cmd_vel_in') $ATTACK_GATE; $ATTACKER_RUN" C-m

cat <<EOF
Started tmux session: $SESSION
Mode: $MODE

Attach:
  tmux attach -t $SESSION

Stop demo:
  tmux kill-session -t $SESSION

Pane layout:
  0 ROBOT_GAZEBO_WSL_HOST
  1 CONTROLLER_DOCKER
  2 ATTACKER_DOCKER
EOF

tmux attach -t "$SESSION"
