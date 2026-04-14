#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_compose.sh"
COMPOSE_CMD="$(resolve_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
$COMPOSE_CMD up -d --build

echo "Containers are up: tb3-controller, tb3-attacker"
