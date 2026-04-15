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
printf '\\033[1;33m%s\\033[0m\\n' '============================================================'
printf '\\033[1;33m%s\\033[0m\\n' '$title'
printf '\\033[1;33m%s\\033[0m\\n' '============================================================'
printf 'Role: %s\\n' '$role'
printf '\\n'
EOF
}

gazebo_cmd() {
  cat <<'EOF'
echo 'Starting secure Gazebo + ros_gz_bridge + cmd_vel_relay.'
./scripts/setup/run_wsl_gazebo_secure.sh
EOF
}

controller_cmd() {
  cat <<'EOF'
sleep 8
echo 'Starting secure containers in BRIDGE mode...'
# We use the bridge compose file for this experiment
export COMPOSE_FILE=deployment/wsl_docker/docker-compose.bridge.yml
export ROS_SECURITY_ENABLE=true
sudo -E docker compose up -d
sleep 2
echo 'Starting controller...'
sudo -E ./scripts/wsl_docker/run_controller.sh
EOF
}

attacker_cmd() {
  local gate
  if [[ "$ATTACK_DELAY" == "0" ]]; then
    gate="read -r -p 'Wait for robot movement, then press Enter to apply 200ms network delay... '"
  else
    gate="sleep '$ATTACK_DELAY'"
  fi

  cat <<EOF
$gate
echo 'Applying network delay inside attacker container...'
sudo ./scripts/wsl_docker/exec_attacker.sh bash /workspace/project/scripts/wsl_docker/mitm_delay_attack.sh 200 50
EOF
}

monitor_cmd() {
  cat <<'EOF'
echo 'Monitoring network latency and traffic...'
watch -n 1 'printf "Docker Container IPs:\n"; docker inspect -f "{{.Name}}: {{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}" tb3-controller-bridge tb3-attacker-bridge 2>/dev/null || true; printf "\nTraffic Control Status (Attacker):\n"; docker exec tb3-attacker-bridge tc qdisc show dev eth0 2>/dev/null || true'
EOF
}

tmux new-session -d -s "$SESSION" -n "sros2-mitm" -c "$PROJECT_ROOT"
tmux split-window -h -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.1" -c "$PROJECT_ROOT"
tmux select-layout -t "$SESSION:0" tiled >/dev/null

tmux select-pane -t "$SESSION:0.0" -T "ROBOT_SIM"
tmux select-pane -t "$SESSION:0.1" -T "SECURE_CONTROLLER"
tmux select-pane -t "$SESSION:0.2" -T "MITM_ATTACKER"
tmux select-pane -t "$SESSION:0.3" -T "NETWORK_MONITOR"

tmux send-keys -t "$SESSION:0.0" "$(banner_cmd 'ROLE 1: SECURE ROBOT' 'WSL host Gazebo.') $(gazebo_cmd)" C-m
tmux send-keys -t "$SESSION:0.1" "$(banner_cmd 'ROLE 2: SECURE CONTROLLER' 'Docker (Bridge Mode).') $(controller_cmd)" C-m
tmux send-keys -t "$SESSION:0.2" "$(banner_cmd 'ROLE 3: MITM ATTACKER' 'Applying tc delay.') $(attacker_cmd)" C-m
tmux send-keys -t "$SESSION:0.3" "$(banner_cmd 'ROLE 4: NETWORK MONITOR' 'Inspect bridge status.') $(monitor_cmd)" C-m

tmux set-option -t "$SESSION" pane-border-status top >/dev/null
tmux set-option -t "$SESSION" pane-border-format " #{pane_index}: #{pane_title} " >/dev/null

echo "Started MITM Delay Demo: $SESSION"
tmux attach-session -t "$SESSION"
