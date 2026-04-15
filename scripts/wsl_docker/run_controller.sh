#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_compose.sh"
COMPOSE_CMD="$(resolve_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
CONTROLLER_PATTERN="${CONTROLLER_PATTERN:-forward}"
CONTROLLER_LINEAR_SPEED="${CONTROLLER_LINEAR_SPEED:-0.2}"
CONTROLLER_ANGULAR_SPEED="${CONTROLLER_ANGULAR_SPEED:-0.8}"
CONTROLLER_SQUARE_SIDE_SECONDS="${CONTROLLER_SQUARE_SIDE_SECONDS:-4.0}"
CONTROLLER_SQUARE_TURN_SECONDS="${CONTROLLER_SQUARE_TURN_SECONDS:-1.96}"

$COMPOSE_CMD exec \
  -e CONTROLLER_PATTERN="$CONTROLLER_PATTERN" \
  -e CONTROLLER_LINEAR_SPEED="$CONTROLLER_LINEAR_SPEED" \
  -e CONTROLLER_ANGULAR_SPEED="$CONTROLLER_ANGULAR_SPEED" \
  -e CONTROLLER_SQUARE_SIDE_SECONDS="$CONTROLLER_SQUARE_SIDE_SECONDS" \
  -e CONTROLLER_SQUARE_TURN_SECONDS="$CONTROLLER_SQUARE_TURN_SECONDS" \
  controller bash -lc \
  "source /opt/ros/jazzy/setup.bash && mkdir -p /workspace/project/logs/wsl_docker && python3 /workspace/project/scripts/wsl_docker/controller_forward_pub.py --ros-args --enclave /controller 2>&1 | tee -a /workspace/project/logs/wsl_docker/controller_pub.log"
