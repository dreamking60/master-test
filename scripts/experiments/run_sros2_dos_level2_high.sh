#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DOS_LABEL="level2_high"
export DOS_PAYLOAD_SIZE="${DOS_PAYLOAD_SIZE:-1400}"
export DOS_THREADS_PER_PORT="${DOS_THREADS_PER_PORT:-16}"
export DOS_RATE="${DOS_RATE:-0}"
export ATTACK_SECONDS="${ATTACK_SECONDS:-60}"
export BASELINE_SECONDS="${BASELINE_SECONDS:-15}"
export AFTER_SECONDS="${AFTER_SECONDS:-12}"

exec "$SCRIPT_DIR/run_log_driven_validation.sh" sros2-dos
