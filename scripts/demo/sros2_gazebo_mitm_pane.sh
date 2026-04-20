#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ATTACK_DELAY_MS="${ATTACK_DELAY_MS:-500}"
ATTACK_LOSS_PERCENT="${ATTACK_LOSS_PERCENT:-5}"
ATTACK_DURATION="${ATTACK_DURATION:-45}"
GAZEBO_UDP_PORT="${GAZEBO_UDP_PORT:-15000}"
SECURITY_KEYSTORE="$PROJECT_ROOT/deployment/wsl_docker/keys"

cd "$PROJECT_ROOT"

banner() {
  local title="$1"
  local role="$2"
  clear
  printf '\033[1;36m%s\033[0m\n' '============================================================'
  printf '\033[1;36m%s\033[0m\n' "$title"
  printf '\033[1;36m%s\033[0m\n' '============================================================'
  printf 'Role: %s\n' "$role"
  printf 'Mode: SROS2 protected controller -> robot-gateway, visualized in WSL Gazebo\n'
  printf 'Security claim: MITM can delay/drop, but cannot validly rewrite DDS commands.\n'
  printf '\n'
}

require_sros2_artifacts() {
  local missing=0
  for enclave in gazebo mitm_controller mitm_robot; do
    if [ ! -f "$SECURITY_KEYSTORE/enclaves/$enclave/permissions.p7s" ]; then
      echo "Missing SROS2 permissions: /$enclave" >&2
      missing=1
    fi
  done
  if [ "$missing" -ne 0 ]; then
    echo "Run: sudo ./scripts/wsl_docker/init_sros2_docker.sh" >&2
    exit 1
  fi
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
  gazebo)
    banner 'ROLE 1: SECURE WSL GAZEBO ROBOT' 'Gazebo GUI + secure UDP command receiver in /gazebo enclave'
    require_sros2_artifacts
    mkdir -p logs/experiments/03_network_mitm
    rm -f logs/experiments/03_network_mitm/mitm_tamper_active

    set +u
    source ./scripts/setup/source_wsl_ros_env.sh
    set -u
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce
    export ROS_SECURITY_KEYSTORE="$SECURITY_KEYSTORE"
    export ROS_SECURITY_ENCLAVE_OVERRIDE=/gazebo

    echo "Starting secure UDP receiver on port $GAZEBO_UDP_PORT..."
    GAZEBO_UDP_PORT="$GAZEBO_UDP_PORT" python3 ./scripts/setup/udp_cmd_vel_receiver.py --ros-args --enclave /gazebo \
      2>&1 | tee logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.log &
    echo $! > logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.pid

    echo 'Starting secure Gazebo flat world...'
    ./scripts/setup/run_wsl_gazebo_secure.sh
    ;;

  controller)
    banner 'ROLE 2: SECURE CONTROLLER' 'Publishes encrypted/authenticated straight commands on /mitm_cmd'
    require_sros2_artifacts
    sudo ./scripts/wsl_docker/mitm_secure_up.sh
    echo
    sudo ./scripts/wsl_docker/mitm_exec.sh controller "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo 'Starting SROS2 command publisher for Gazebo path...'
    secure_exec controller \
      "source /opt/ros/jazzy/setup.bash && MITM_CMD_RATE_HZ=10 MITM_CMD_LINEAR_X=0.2 MITM_CMD_ANGULAR_Z=0.0 python3 /workspace/project/scripts/wsl_docker/mitm_controller_cmd_pub.py --ros-args --enclave /mitm_controller"
    ;;

  robot)
    banner 'ROLE 3: SECURE ROBOT GATEWAY' 'Subscribes to encrypted /mitm_cmd and forwards unchanged commands to WSL Gazebo over UDP'
    require_sros2_artifacts
    wait_for_container tb3-mitm-robot
    sudo ./scripts/wsl_docker/mitm_exec.sh robot "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo 'Starting secure robot UDP forwarder to WSL host gateway 172.28.0.1...'
    echo 'Tamper is intentionally disabled in this SROS2 demo.'
    secure_exec robot \
      "source /opt/ros/jazzy/setup.bash && GAZEBO_UDP_HOST=172.28.0.1 GAZEBO_UDP_PORT='$GAZEBO_UDP_PORT' MITM_TAMPER_ENABLE=0 MITM_TAMPER_FLAG_FILE=/tmp/sros2_gazebo_mitm_no_tamper_flag python3 /workspace/project/scripts/wsl_docker/mitm_robot_udp_forwarder.py --ros-args --enclave /mitm_robot"
    ;;

  attacker)
    banner 'ROLE 4: NETWORK MITM ATTACKER' 'ARP poisoner + forwarding delay/loss injector; no SROS2 keys for command tampering'
    wait_for_container tb3-mitm-attacker
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "hostname; ip -br addr; ip route; ip neigh; env | grep -E 'ROS_SECURITY|ROS_SECURITY_ENCLAVE|ROS_DOMAIN' | sort"
    echo
    echo "Attack settings: delay=${ATTACK_DELAY_MS}ms loss=${ATTACK_LOSS_PERCENT}% duration=${ATTACK_DURATION}s"
    read -r -p 'Wait until Gazebo robot moves straight, then press Enter to start SROS2 MITM delay/drop... '
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker \
      "bash /workspace/project/scripts/wsl_docker/mitm_enable_forward_delay.sh '$ATTACK_DELAY_MS' '$ATTACK_LOSS_PERCENT'"
    sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration "$ATTACK_DURATION" --log-dir /tmp
    echo 'Cleaning attacker qdisc...'
    sudo ./scripts/wsl_docker/mitm_exec.sh attacker "tc qdisc del dev eth0 root 2>/dev/null || true; tc qdisc show dev eth0"
    read -r -p 'SROS2 Gazebo MITM attack finished. Press Enter to keep pane open... ' _
    ;;

  monitor)
    banner 'ROLE 5: MONITOR' 'ARP/qdisc and secure Gazebo UDP receiver logs'
    watch -n 1 "printf 'Containers:\n'; sudo docker ps --format 'table {{.Names}}\t{{.Networks}}\t{{.Status}}' | grep -E 'NAMES|tb3-mitm' || true; printf '\nController ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh controller 'ip neigh' 2>/dev/null || true; printf '\nRobot ARP:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh robot 'ip neigh' 2>/dev/null || true; printf '\nAttacker qdisc/ip_forward:\n'; sudo ./scripts/wsl_docker/mitm_exec.sh attacker 'cat /proc/sys/net/ipv4/ip_forward; tc qdisc show dev eth0' 2>/dev/null || true; printf '\nSROS2 Gazebo UDP receiver:\n'; tail -n 8 logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.log 2>/dev/null || true"
    ;;

  *)
    echo "Usage: $0 <gazebo|controller|robot|attacker|monitor>" >&2
    exit 1
    ;;
esac
