#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BRIDGE_COMPOSE_FILE="$PROJECT_ROOT/deployment/wsl_docker/docker-compose.bridge.yml"
ATTACK_DELAY="${ATTACK_DELAY:-0}"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

cd "$PROJECT_ROOT"

banner() {
  local title="$1"
  local role="$2"
  clear
  printf '\033[1;33m%s\033[0m\n' '============================================================'
  printf '\033[1;33m%s\033[0m\n' "$title"
  printf '\033[1;33m%s\033[0m\n' '============================================================'
  printf 'Role: %s\n' "$role"
  printf '\n'
}

case "$ROLE" in
  robot)
    banner 'ROLE 1: SECURE ROBOT' 'WSL host Gazebo.'
    export ROS_STATIC_PEERS='172.18.0.10;172.18.0.50'
    echo "ROS_STATIC_PEERS=$ROS_STATIC_PEERS"
    echo 'Starting secure Gazebo + ros_gz_bridge + cmd_vel_relay.'
    ./scripts/setup/run_wsl_gazebo_secure.sh
    ;;

  controller)
    banner 'ROLE 2: SECURE CONTROLLER' 'Docker bridge-mode controller.'
    echo 'Waiting 8s for Gazebo startup...'
    sleep 8
    export ROS_STATIC_PEERS='172.18.0.1'
    echo "ROS_STATIC_PEERS=$ROS_STATIC_PEERS"
    echo 'Starting secure containers in BRIDGE mode...'
    sudo -E docker compose -f "$BRIDGE_COMPOSE_FILE" up -d --build --force-recreate
    echo
    echo 'Bridge containers:'
    sudo docker compose -f "$BRIDGE_COMPOSE_FILE" ps
    echo
    echo 'Starting controller publisher in tb3-controller-bridge...'
    sudo -E env \
      COMPOSE_FILE="$BRIDGE_COMPOSE_FILE" \
      ROS_STATIC_PEERS="$ROS_STATIC_PEERS" \
      CONTROLLER_PATTERN=forward \
      CONTROLLER_LINEAR_SPEED=0.2 \
      ./scripts/wsl_docker/run_controller.sh
    ;;

  attacker)
    banner 'ROLE 3: ATTACKER' 'Docker bridge-mode attacker trying to inject turn commands.'
    if [[ "$ATTACK_DELAY" == "0" ]]; then
      read -r -p 'Wait for straight robot movement, then press Enter to start turn-command injection... '
    else
      echo "Attacker will start after ${ATTACK_DELAY}s..."
      sleep "$ATTACK_DELAY"
    fi
    echo
    export ROS_STATIC_PEERS='172.18.0.1'
    echo "ROS_STATIC_PEERS=$ROS_STATIC_PEERS"
    echo 'Waiting for tb3-attacker-bridge to be running...'
    for _ in $(seq 1 60); do
      if sudo docker inspect -f '{{.State.Running}}' tb3-attacker-bridge 2>/dev/null | grep -qx true; then
        break
      fi
      sleep 1
    done
    if ! sudo docker inspect -f '{{.State.Running}}' tb3-attacker-bridge 2>/dev/null | grep -qx true; then
      echo 'ERROR: tb3-attacker-bridge is not running. Check the controller pane compose startup.' >&2
      exit 1
    fi
    echo 'Starting attacker turn publisher in tb3-attacker-bridge...'
    sudo -E env \
      COMPOSE_FILE="$BRIDGE_COMPOSE_FILE" \
      ROS_STATIC_PEERS="$ROS_STATIC_PEERS" \
      ./scripts/wsl_docker/run_attacker.sh
    ;;

  monitor)
    banner 'ROLE 4: NETWORK MONITOR' 'Inspect bridge containers and command logs.'
    echo 'Monitoring bridge containers and command logs...'
    watch -n 1 "printf 'Docker Container IPs:\n'; sudo docker inspect -f '{{.Name}}: {{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' tb3-controller-bridge tb3-attacker-bridge 2>/dev/null || true; printf '\nBridge compose services:\n'; sudo docker compose -f '$BRIDGE_COMPOSE_FILE' ps 2>/dev/null || true; printf '\nRecent controller logs:\n'; tail -n 5 '$PROJECT_ROOT/logs/wsl_docker/controller_pub.log' 2>/dev/null || true; printf '\nRecent attacker logs:\n'; tail -n 8 '$PROJECT_ROOT/logs/wsl_docker/attacker_pub.log' 2>/dev/null || true"
    ;;

  *)
    echo "Usage: $0 <robot|controller|attacker|monitor>" >&2
    exit 1
    ;;
esac
