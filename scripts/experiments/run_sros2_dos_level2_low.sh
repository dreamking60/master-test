#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DOS_LABEL="level2_low"
export DOS_PAYLOAD_SIZE="${DOS_PAYLOAD_SIZE:-1200}"
export DOS_THREADS_PER_PORT="${DOS_THREADS_PER_PORT:-6}"
export DOS_RATE="${DOS_RATE:-0}"
export ATTACK_SECONDS="${ATTACK_SECONDS:-35}"
export BASELINE_SECONDS="${BASELINE_SECONDS:-15}"
export AFTER_SECONDS="${AFTER_SECONDS:-8}"

exec "$SCRIPT_DIR/run_log_driven_validation.sh" sros2-dos
