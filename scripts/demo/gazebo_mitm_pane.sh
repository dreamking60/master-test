#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ATTACK_DELAY_MS="${ATTACK_DELAY_MS:-500}"
ATTACK_LOSS_PERCENT="${ATTACK_LOSS_PERCENT:-5}"
ATTACK_DURATION="${ATTACK_DURATION:-45}"
GAZEBO_UDP_PORT="${GAZEBO_UDP_PORT:-15000}"
MITM_TAMPER_ENABLE="${MITM_TAMPER_ENABLE:-0}"
MITM_TAMPER_ANGULAR_Z="${MITM_TAMPER_ANGULAR_Z:-1.2}"
MITM_TAMPER_FLAG_FILE="${MITM_TAMPER_FLAG_FILE:-/workspace/project/logs/experiments/03_network_mitm/mitm_tamper_active}"
HOST_TAMPER_FLAG_FILE="${PROJECT_ROOT}/logs/experiments/03_network_mitm/mitm_tamper_active"

cd "$PROJECT_ROOT"

banner() {
  local title="$1"
  local role="$2"
  clear
  printf '\033[1;35m%s\033[0m\n' '============================================================'
  printf '\033[1;35m%s\033[0m\n' "$title"
  printf '\033[1;35m%s\033[0m\n' '============================================================'
  printf 'Role: %s\n' "$role"
  printf 'Path: controller -> attacker MITM -> robot-gateway -> UDP -> WSL Gazebo\n'
  printf '\n'
}

case "$ROLE" in
  gazebo)
    banner 'ROLE 1: WSL GAZEBO ROBOT' 'Gazebo GUI + UDP command receiver'
    mkdir -p logs/experiments/03_network_mitm
    rm -f "$HOST_TAMPER_FLAG_FILE"
    echo "Starting UDP receiver on port $GAZEBO_UDP_PORT in background..."
    # ROS setup scripts are not nounset-safe.
    set +u
    source ./scripts/setup/source_wsl_ros_env.sh
    set -u
    GAZEBO_UDP_PORT="$GAZEBO_UDP_PORT" python3 ./scripts/setup/udp_cmd_vel_receiver.py \
      2>&1 | tee logs/experiments/03_network_mitm/gazebo_udp_receiver.log &
    echo $! > logs/experiments/03_network_mitm/gazebo_udp_receiver.pid
    echo 'Starting Gazebo flat world...'
    ./scripts/setup/run_wsl_gazebo.sh
    ;;

  controller)
    banner 'ROLE 2: CONTROLLER' 'Docker MITM lab controller publishes ROS2 commands'
    sudo ./scripts/wsl_docker/mitm_up.sh
    echo
    sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; ip neigh"
    echo
    echo 'Starting ROS2 command publisher for Gazebo path...'
    sudo ./scripts/wsl_docker/mitm_exec.sh controller \
      "source /opt/ros/jazzy/setup.bash && MITM_CMD_RATE_HZ=10 MITM_CMD_LINEAR_X=0.2 MITM_CMD_ANGULAR_Z=0.0 python3 /workspace/project/scripts/wsl_docker/mitm_controller_cmd_pub.py"
    ;;

  robot)
    banner 'ROLE 3: ROBOT GATEWAY' 'Docker target endpoint forwards ROS2 commands to WSL Gazebo over UDP'
    sleep 5
    sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; ip neigh"
    echo
    echo 'Starting robot UDP forwarder to WSL host gateway 172.28.0.1...'
    echo "MITM_TAMPER_ENABLE=$MITM_TAMPER_ENABLE MITM_TAMPER_ANGULAR_Z=$MITM_TAMPER_ANGULAR_Z"
    echo "MITM_TAMPER_FLAG_FILE=$MITM_TAMPER_FLAG_FILE"
    sudo ./scripts/wsl_docker/mitm_exec.sh robot \
      "source /opt/ros/jazzy/setup.bash && GAZEBO_UDP_HOST=172.28.0.1 GAZEBO_UDP_PORT='$GAZEBO_UDP_PORT' MITM_TAMPER_ENABLE='$MITM_TAMPER_ENABLE' MITM_TAMPER_ANGULAR_Z='$MITM_TAMPER_ANGULAR_Z' MITM_TAMPER_FLAG_FILE='$MITM_TAMPER_FLAG_FILE' python3 /workspace/project/scripts/wsl_docker/mitm_robot_udp_forwarder.py"
    ;;

  attacker)
    banner 'ROLE 4: MITM ATTACKER' 'ARP poisoner + forwarding delay/loss injector'
    sleep 8
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; ip neigh"
    echo
    echo "Attack settings: delay=${ATTACK_DELAY_MS}ms loss=${ATTACK_LOSS_PERCENT}% duration=${ATTACK_DURATION}s"
    read -r -p 'Wait until Gazebo robot moves straight, then press Enter to start ARP MITM... '
    mkdir -p "$(dirname "$HOST_TAMPER_FLAG_FILE")"
    echo "active $(date --iso-8601=seconds)" > "$HOST_TAMPER_FLAG_FILE"
    echo "Tamper flag enabled: $HOST_TAMPER_FLAG_FILE"
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker \
      "bash /workspace/project/scripts/wsl_docker/mitm_enable_forward_delay.sh '$ATTACK_DELAY_MS' '$ATTACK_LOSS_PERCENT'"
    sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration "$ATTACK_DURATION" --log-dir /tmp
    rm -f "$HOST_TAMPER_FLAG_FILE"
    echo 'Tamper flag disabled.'
    echo 'Cleaning attacker qdisc...'
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true; tc qdisc show dev eth0"
    read -r -p 'Gazebo MITM attack finished. Press Enter to keep pane open... ' _
    ;;

  monitor)
    banner 'ROLE 5: MONITOR' 'ARP/qdisc and Gazebo receiver logs'
    watch -n 1 "printf 'Containers:\n'; sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}' | grep -E 'NAMES|tb3-mitm' || true; printf '\nController ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh controller 'ip neigh' 2>/dev/null || true; printf '\nRobot ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh robot 'ip neigh' 2>/dev/null || true; printf '\nAttacker qdisc/ip_forward:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0' 2>/dev/null || true; printf '\nGazebo UDP receiver:\n'; tail -n 8 logs/experiments/03_network_mitm/gazebo_udp_receiver.log 2>/dev/null || true"
    ;;

  *)
    echo "Usage: $0 <gazebo|controller|robot|attacker|monitor>" >&2
    exit 1
    ;;
esac
