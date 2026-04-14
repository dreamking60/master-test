#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

# Start containers first
sudo "$SCRIPT_DIR/up.sh"

# Run normal controller in foreground
sudo "$SCRIPT_DIR/run_controller.sh"
