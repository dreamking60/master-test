#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ATTACK_DELAY_MS="${ATTACK_DELAY_MS:-500}"
ATTACK_LOSS_PERCENT="${ATTACK_LOSS_PERCENT:-5}"
ATTACK_DURATION="${ATTACK_DURATION:-45}"

cd "$PROJECT_ROOT"

banner() {
  local title="$1"
  local role="$2"
  clear
  printf '\033[1;32m%s\033[0m\n' '============================================================'
  printf '\033[1;32m%s\033[0m\n' "$title"
  printf '\033[1;32m%s\033[0m\n' '============================================================'
  printf 'Role: %s\n' "$role"
  printf 'Mode: SROS2 protected ROS2 command traffic + ARP MITM availability attack\n'
  printf '\n'
}

secure_exec() {
  local role="$1"
  shift
  sudo env ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce \
    ./scripts/wsl_docker/mitm_exec.sh "$role" "$*"
}

wait_for_container() {
  local name="$1"
  echo "Waiting for $name to be running..."
  for _ in $(seq 1 90); do
    if sudo docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null | grep -qx true; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: $name is not running. Check the controller pane startup." >&2
  exit 1
}

case "$ROLE" in
  controller)
    banner 'ROLE 1: SECURE CONTROLLER' 'Publishes encrypted/authenticated /mitm_cmd'
    sudo ./scripts/wsl_docker/mitm_secure_up.sh
    echo
    sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo 'Starting SROS2 command publisher...'
    secure_exec controller "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_controller_cmd_pub.py --ros-args --enclave /mitm_controller"
    ;;

  robot)
    banner 'ROLE 2: SECURE ROBOT ENDPOINT' 'Subscribes to encrypted/authenticated /mitm_cmd'
    wait_for_container tb3-mitm-robot
    sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo 'Starting SROS2 command subscriber...'
    secure_exec robot "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_robot_cmd_sub.py --ros-args --enclave /mitm_robot"
    ;;

  attacker)
    banner 'ROLE 3: NETWORK MITM ATTACKER' 'Can ARP poison/delay/drop but cannot decrypt or validly modify SROS2 payloads'
    wait_for_container tb3-mitm-attacker
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo "Attack settings: delay=${ATTACK_DELAY_MS}ms loss=${ATTACK_LOSS_PERCENT}% duration=${ATTACK_DURATION}s"
    read -r -p 'Wait until SROS2 subscriber receives commands, then press Enter to start ARP MITM... '
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker \
      "bash /workspace/project/scripts/wsl_docker/mitm_enable_forward_delay.sh '$ATTACK_DELAY_MS' '$ATTACK_LOSS_PERCENT'"
    sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration "$ATTACK_DURATION" --log-dir /tmp
    echo 'Cleaning attacker qdisc...'
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true; tc qdisc show dev eth0"
    read -r -p 'SROS2 MITM attack finished. Press Enter to keep pane open... ' _
    ;;

  monitor)
    banner 'ROLE 4: MONITOR' 'ARP/qdisc and DDS packet visibility'
    watch -n 1 "printf 'Containers:\n'; sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}' | grep -E 'NAMES|tb3-mitm' || true; printf '\nController ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh controller 'ip neigh' 2>/dev/null || true; printf '\nRobot ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh robot 'ip neigh' 2>/dev/null || true; printf '\nAttacker qdisc/ip_forward:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0' 2>/dev/null || true"
    ;;

  *)
    echo "Usage: $0 <controller|robot|attacker|monitor>" >&2
    exit 1
    ;;
esac
