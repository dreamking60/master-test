#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ROS setup scripts are not always nounset-safe.
set +u
source "$SCRIPT_DIR/source_wsl_ros_env.sh"
set -u

exec "$SCRIPT_DIR/run.sh"
