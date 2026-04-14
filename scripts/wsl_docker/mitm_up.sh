#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$SCRIPT_DIR/_mitm_compose.sh"
COMPOSE_CMD="$(resolve_mitm_compose_cmd)"

cd "$PROJECT_ROOT/deployment/wsl_docker"
$COMPOSE_CMD up -d --build

echo "MITM lab containers are up:"
echo "  tb3-mitm-controller 172.28.0.10"
echo "  tb3-mitm-robot      172.28.0.20"
echo "  tb3-mitm-attacker   172.28.0.50"
