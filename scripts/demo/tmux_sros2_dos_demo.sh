#!/usr/bin/env bash
# SROS2 UDP DoS demo.
#
# This script intentionally keeps the secure controller path identical to
# Experiment 02. The attacker starts only after manual confirmation by default,
# so a normal secure baseline can be observed first.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SESSION="${SESSION:-sros2_dos_demo}"
ATTACK_DELAY="${ATTACK_DELAY:-0}"

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

if [[ "${SKIP_CLEANUP:-0}" != "1" ]]; then
  "$SCRIPT_DIR/cleanup_all_experiments.sh"
fi

if [ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/controller/permissions.p7s" ]; then
  echo "SROS2 keys not found. Generating..."
  sudo "$PROJECT_ROOT/scripts/wsl_docker/init_sros2_docker.sh"
fi

tmux_env_exports() {
  local name
  for name in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR PULSE_SERVER DBUS_SESSION_BUS_ADDRESS WSL_INTEROP WSL_DISTRO_NAME; do
    if [[ -n "${!name:-}" ]]; then
      printf 'export %s=%q\n' "$name" "${!name}"
    fi
  done
}

banner_cmd() {
  local title="$1"
  local role="$2"
  cat <<EOF
clear
cd "$PROJECT_ROOT"
$(tmux_env_exports)
printf '\\033[1;36m%s\\033[0m\\n' '============================================================'
printf '\\033[1;36m%s\\033[0m\\n' '$title'
printf '\\033[1;36m%s\\033[0m\\n' '============================================================'
printf 'Role: %s\\n' '$role'
printf '\\n'
EOF
}

gazebo_cmd() {
  cat <<'EOF'
touch test.log
echo 'Starting secure Gazebo + ros_gz_bridge + cmd_vel_relay.'
echo 'This pane must use SROS2 enclave /gazebo; otherwise the secure controller cannot move the robot.'
echo
(./scripts/setup/run_wsl_gazebo_secure.sh; echo "[GAZEBO_EXIT] command exited with status $?") &
GAZEBO_PID=$!
sleep 6
echo
echo '--- live test.log ---'
tail -n 120 -f test.log &
TAIL_PID=$!
wait $GAZEBO_PID
STATUS=$?
kill $TAIL_PID 2>/dev/null || true
wait $TAIL_PID 2>/dev/null || true
echo
echo "Gazebo command finished with status $STATUS. Press Enter to keep this pane open."
read -r _
EOF
}

controller_cmd() {
  cat <<'EOF'
sleep 10
echo 'Starting secure Docker containers and controller.'
echo 'Expected baseline before attack: robot follows a square-like path.'
CONTROLLER_PATTERN=square \
CONTROLLER_LINEAR_SPEED=0.2 \
CONTROLLER_ANGULAR_SPEED=0.8 \
CONTROLLER_SQUARE_SIDE_SECONDS=4.0 \
CONTROLLER_SQUARE_TURN_SECONDS=1.96 \
sudo -E ./scripts/wsl_docker/secure_start_controller_stack.sh
EOF
}

attacker_cmd() {
  local gate
  if [[ "$ATTACK_DELAY" == "0" ]]; then
    gate="read -r -p 'Confirm the secure controller is following a square path, then press Enter to start UDP DoS... '"
  else
    gate="echo 'UDP DoS will start after ${ATTACK_DELAY}s...'; sleep '$ATTACK_DELAY'"
  fi

  cat <<EOF
$gate
echo 'Starting UDP flood from attacker container.'
echo 'This tests availability/resource exhaustion, not SROS2 permission bypass.'
sudo ./scripts/wsl_docker/exec_attacker.sh python3 /workspace/project/attack_experiments/scripts/dos_attack_sros2.py --discover-ports --duration 45 --payload-size 1200 --threads-per-port 3
EOF
}

monitor_cmd() {
  cat <<'EOF'
echo 'Monitoring controller container CPU/memory.'
echo 'Use Ctrl-C in this pane to stop monitoring only.'
watch -n 1 'printf "Docker stats:\n"; docker stats --no-stream tb3-controller tb3-attacker 2>/dev/null || true; printf "\nDDS UDP sockets:\n"; ss -lunp 2>/dev/null | grep -E ":(74[0-9][0-9]|75[0-9][0-9])" || true; printf "\nController log:\n"; tail -n 5 logs/wsl_docker/controller_pub.log 2>/dev/null || true'
EOF
}

tmux new-session -d -s "$SESSION" -n "sros2-dos" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "SECURE_ROBOT_GAZEBO"
tmux select-pane -t "$SESSION:0.1" -T "SECURE_CONTROLLER"
tmux select-pane -t "$SESSION:0.2" -T "UDP_DOS_ATTACKER"
tmux select-pane -t "$SESSION:0.3" -T "RESOURCE_MONITOR"
tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

tmux send-keys -t "$SESSION:0.0" "$(banner_cmd 'ROLE 1: SECURE ROBOT / GAZEBO' 'WSL host. Runs Gazebo, bridge, and secure relay with enclave /gazebo.') $(gazebo_cmd)" C-m
tmux send-keys -t "$SESSION:0.1" "$(banner_cmd 'ROLE 2: SECURE CONTROLLER' 'Docker controller using enclave /controller.') $(controller_cmd)" C-m
tmux send-keys -t "$SESSION:0.2" "$(banner_cmd 'ROLE 3: UDP DoS ATTACKER' 'Docker attacker sends UDP packets at DDS/RTPS ports after baseline is confirmed.') $(attacker_cmd)" C-m
tmux send-keys -t "$SESSION:0.3" "$(banner_cmd 'ROLE 4: RESOURCE MONITOR' 'Host-side docker stats for availability impact.') $(monitor_cmd)" C-m

cat <<EOF
Started tmux session: $SESSION

Expected sequence:
  1. Wait for Gazebo to open.
  2. Confirm the controller moves the robot forward.
  3. Press Enter in the attacker pane to start UDP DoS.

Attach:
  tmux attach -t $SESSION

Stop:
  tmux kill-session -t $SESSION
EOF

tmux attach-session -t "$SESSION"
