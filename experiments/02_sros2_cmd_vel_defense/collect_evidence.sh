#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$PROJECT_ROOT/logs/experiments/02_sros2_cmd_vel_defense"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$OUT_DIR/evidence_$STAMP.log"

mkdir -p "$OUT_DIR"

run_section() {
  local title="$1"
  shift
  {
    echo
    echo "## $title"
    echo "\$ $*"
    "$@" 2>&1 || true
  } >> "$OUT"
}

run_shell_section() {
  local title="$1"
  local command="$2"
  {
    echo
    echo "## $title"
    echo "\$ $command"
    bash -lc "$command" 2>&1 || true
  } >> "$OUT"
}

{
  echo "# SROS2 command-defense evidence"
  echo "Timestamp: $STAMP"
  echo "Project: $PROJECT_ROOT"
} > "$OUT"

run_section "Git status" git -C "$PROJECT_ROOT" status --short
run_section "Keystore files" find "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves" -maxdepth 2 \( -type f -o -type l \) -print
run_section "Controller permissions" sed -n '1,220p' "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/controller/permissions.xml"
run_section "Attacker permissions" sed -n '1,220p' "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/attacker/permissions.xml"
run_section "Gazebo permissions" sed -n '1,260p' "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/gazebo/permissions.xml"

run_shell_section "Host /cmd_vel_in topic info" "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_LOCALHOST_ONLY=0 ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce ROS_SECURITY_KEYSTORE='$PROJECT_ROOT/deployment/wsl_docker/keys' ROS_SECURITY_ENCLAVE_OVERRIDE=/gazebo ROS_LOG_DIR='$PROJECT_ROOT/logs'; timeout 5 ros2 topic info /cmd_vel_in -v"
run_shell_section "Host /cmd_vel topic info" "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_LOCALHOST_ONLY=0 ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce ROS_SECURITY_KEYSTORE='$PROJECT_ROOT/deployment/wsl_docker/keys' ROS_SECURITY_ENCLAVE_OVERRIDE=/gazebo ROS_LOG_DIR='$PROJECT_ROOT/logs'; timeout 5 ros2 topic info /cmd_vel -v"
run_shell_section "Recent Gazebo secure log markers" "tail -n 160 '$PROJECT_ROOT/test.log' | grep -E 'security|enclave|cmd_vel_relay|relayed=|ros_gz_bridge|ERROR|WARN|Found security directory' || true"
run_shell_section "Recent controller log markers" "tail -n 120 '$PROJECT_ROOT/logs/wsl_docker/controller_pub.log' || true"
run_shell_section "Recent attacker log markers" "tail -n 120 '$PROJECT_ROOT/logs/wsl_docker/attacker_pub.log' || true"

echo "Evidence written to: $OUT"
