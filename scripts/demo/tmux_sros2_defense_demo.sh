#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSION="${SESSION:-tb3_sros2_defense_demo}" \
  "$SCRIPT_DIR/tmux_three_machine_demo.sh" secure
