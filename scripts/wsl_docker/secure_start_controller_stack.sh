#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo "$SCRIPT_DIR/secure_up.sh"
sudo "$SCRIPT_DIR/run_controller.sh"
