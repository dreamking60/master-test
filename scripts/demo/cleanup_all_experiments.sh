#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "Cleaning previous experiment state..."

for session in tb3_security_demo tb3_sros2_defense_demo tb3_network_mitm_demo; do
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "  killing tmux session: $session"
    tmux kill-session -t "$session" || true
  fi
done

echo "  stopping host-network controller/attacker containers"
(cd "$PROJECT_ROOT" && sudo ./scripts/wsl_docker/down.sh) >/dev/null 2>&1 || true

echo "  stopping MITM bridge lab containers"
(cd "$PROJECT_ROOT" && sudo ./scripts/wsl_docker/mitm_down.sh) >/dev/null 2>&1 || true

echo "  stopping Gazebo/ROS launch leftovers"
pkill -f "ros2 launch .*turtlebot3" 2>/dev/null || true
pkill -f "turtlebot3_empty_world_custom_bridge.launch.py" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "cmd_vel_relay.py" 2>/dev/null || true

if command -v ros2 >/dev/null 2>&1; then
  echo "  resetting ROS2 daemon"
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
fi

echo "Cleanup complete."
