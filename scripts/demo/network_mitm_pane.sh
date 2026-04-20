#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ATTACK_DELAY_MS="${ATTACK_DELAY_MS:-250}"
ATTACK_LOSS_PERCENT="${ATTACK_LOSS_PERCENT:-0}"
ATTACK_DURATION="${ATTACK_DURATION:-60}"

cd "$PROJECT_ROOT"

banner() {
  local title="$1"
  local role="$2"
  clear
  printf '\033[1;33m%s\033[0m\n' '============================================================'
  printf '\033[1;33m%s\033[0m\n' "$title"
  printf '\033[1;33m%s\033[0m\n' '============================================================'
  printf 'Role: %s\n' "$role"
  printf 'Lab : Docker bridge/NAT network 172.28.0.0/24\n'
  printf '\n'
}

case "$ROLE" in
  controller)
    banner 'ROLE 1: CONTROLLER ENDPOINT' 'Legitimate sender: 172.28.0.10'
    sudo ./scripts/wsl_docker/mitm_up.sh
    echo
    sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; echo; echo 'Initial ARP table:'; ip neigh"
    echo
    echo 'Starting background ping to robot. Watch RTT before/during/after MITM.'
    sudo ./scripts/wsl_docker/mitm_exec.sh controller "ping 172.28.0.20" &
    echo
    echo 'Starting ROS2 command publisher with message timestamps.'
    sudo ./scripts/wsl_docker/mitm_exec.sh controller \
      "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_controller_cmd_pub.py"
    ;;

  robot)
    banner 'ROLE 2: ROBOT ENDPOINT' 'Target receiver: 172.28.0.20'
    sleep 5
    sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; echo; echo 'Initial ARP table:'; ip neigh"
    echo
    echo 'Starting background packet view for ARP, ICMP, and DDS/RTPS UDP ports.'
    sudo ./scripts/wsl_docker/mitm_exec.sh robot "tcpdump -n -i eth0 'arp or icmp or udp portrange 7400-7600'" &
    echo
    echo 'Starting ROS2 command subscriber. Watch latency before/during/after MITM.'
    sudo ./scripts/wsl_docker/mitm_exec.sh robot \
      "source /opt/ros/jazzy/setup.bash && python3 /workspace/project/scripts/wsl_docker/mitm_robot_cmd_sub.py"
    ;;

  attacker)
    banner 'ROLE 3: MITM ATTACKER' 'ARP poisoner and forwarding relay: 172.28.0.50'
    sleep 7
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; echo; echo 'Initial ARP table:'; ip neigh"
    echo
    echo "Attack settings: delay=${ATTACK_DELAY_MS}ms loss=${ATTACK_LOSS_PERCENT}% duration=${ATTACK_DURATION}s"
    read -r -p 'Press Enter to enable forwarding, add delay, and start ARP MITM... '
    echo
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker \
      "bash /workspace/project/scripts/wsl_docker/mitm_enable_forward_delay.sh '$ATTACK_DELAY_MS' '$ATTACK_LOSS_PERCENT'"
    echo
    sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration "$ATTACK_DURATION"
    echo
    echo 'Cleaning attacker qdisc...'
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true; tc qdisc show dev eth0"
    echo
    read -r -p 'MITM attack finished. Press Enter to keep attacker pane open... ' _
    ;;

  monitor)
    banner 'ROLE 4: NETWORK MONITOR' 'Container state and ARP evidence'
    watch -n 1 "printf 'Containers:\n'; sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}' | grep -E 'NAMES|tb3-mitm' || true; printf '\nController ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh controller 'ip neigh' 2>/dev/null || true; printf '\nRobot ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh robot 'ip neigh' 2>/dev/null || true; printf '\nAttacker qdisc/ip_forward:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0' 2>/dev/null || true; printf '\nROS2 topic visibility:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh robot 'source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic list' 2>/dev/null || true"
    ;;

  *)
    echo "Usage: $0 <controller|robot|attacker|monitor>" >&2
    exit 1
    ;;
esac
