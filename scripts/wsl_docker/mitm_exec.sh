#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <controller|robot|attacker> <command...>" >&2
  exit 1
fi

ROLE="$1"
shift

case "$ROLE" in
  controller) SERVICE="mitm-controller" ;;
  robot) SERVICE="mitm-robot" ;;
  attacker) SERVICE="mitm-attacker" ;;
  *)
    echo "ERROR: role must be controller, robot, or attacker" >&2
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_mitm_compose.sh"
COMPOSE_CMD="$(resolve_mitm_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
$COMPOSE_CMD exec "$SERVICE" bash -lc "$*"
