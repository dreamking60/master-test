#!/usr/bin/env bash
set -euo pipefail

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 <command>"
  echo "Example: $0 ros2 topic list"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_compose.sh"
COMPOSE_CMD="$(resolve_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
$COMPOSE_CMD exec controller bash -lc "source /opt/ros/jazzy/setup.bash && $*"
