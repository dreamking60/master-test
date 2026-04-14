#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$PROJECT_ROOT/logs/wsl_docker"
OUT_FILE="$OUT_DIR/diagnostics_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "$OUT_DIR"
touch "$OUT_FILE"

run_cmd() {
  local title="$1"
  local cmd="$2"
  echo "=== $title ==="
  bash -lc "$cmd" 2>&1 || true
  echo
}

{
  source "$PROJECT_ROOT/scripts/setup/source_wsl_ros_env.sh" >/dev/null 2>&1 || true

  run_cmd "Host Env" "env | grep -E '^ROS_|^RMW_' | sort"
  run_cmd "Docker PS" "docker ps --format 'table {{.Names}}\\t{{.Status}}\\t{{.Image}}' || sudo -n docker ps --format 'table {{.Names}}\\t{{.Status}}\\t{{.Image}}'"
  run_cmd "Controller Env" "\"$SCRIPT_DIR/exec_controller.sh\" \"env | grep -E '^ROS_|^RMW_' | sort\""
  run_cmd "Attacker Env" "\"$SCRIPT_DIR/exec_attacker.sh\" \"env | grep -E '^ROS_|^RMW_' | sort\""
  run_cmd "Host Topic Info /cmd_vel" "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; ros2 topic info /cmd_vel -v"
  run_cmd "Host Node List" "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; ros2 node list"
  run_cmd "Recent test.log" "tail -n 120 \"$PROJECT_ROOT/test.log\""
  run_cmd "Recent controller_pub.log" "tail -n 80 \"$OUT_DIR/controller_pub.log\""
  run_cmd "Recent attacker_pub.log" "tail -n 80 \"$OUT_DIR/attacker_pub.log\""

} | tee "$OUT_FILE"

echo
echo "Diagnostics written to: $OUT_FILE"
