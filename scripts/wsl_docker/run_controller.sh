#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_compose.sh"
COMPOSE_CMD="$(resolve_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
$COMPOSE_CMD exec controller bash -lc \
  "source /opt/ros/jazzy/setup.bash && mkdir -p /workspace/project/logs/wsl_docker && python3 /workspace/project/scripts/wsl_docker/controller_forward_pub.py --ros-args --enclave /controller 2>&1 | tee -a /workspace/project/logs/wsl_docker/controller_pub.log"
