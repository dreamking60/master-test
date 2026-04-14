#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KEYS_DIR="$PROJECT_ROOT/deployment/wsl_docker/keys"
POLICY="$PROJECT_ROOT/config/sros2_wsl_docker_policy.xml"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

python3 - "$POLICY" <<'PY'
from pathlib import Path
import sys
from sros2.policy import load_policy

load_policy(Path(sys.argv[1]))
print("policy xml ok")
PY

controller_perm="$KEYS_DIR/enclaves/controller/permissions.xml"
attacker_perm="$KEYS_DIR/enclaves/attacker/permissions.xml"
gazebo_perm="$KEYS_DIR/enclaves/gazebo/permissions.xml"

for file in "$controller_perm" "$attacker_perm" "$gazebo_perm"; do
  if [ ! -f "$file" ]; then
    echo "ERROR: missing permissions file: $file" >&2
    exit 1
  fi
done

if ! grep -q '<topic>rt/cmd_vel_in</topic>' "$controller_perm"; then
  echo "ERROR: controller is not allowed to publish rt/cmd_vel_in" >&2
  exit 1
fi

if grep -q '<topic>rt/cmd_vel_in</topic>' "$attacker_perm"; then
  echo "ERROR: attacker unexpectedly has rt/cmd_vel_in permission" >&2
  exit 1
fi

if ! grep -q '<topic>rt/cmd_vel_in</topic>' "$gazebo_perm"; then
  echo "ERROR: gazebo/relay is not allowed to subscribe rt/cmd_vel_in" >&2
  exit 1
fi

if ! grep -q '<topic>rt/cmd_vel</topic>' "$gazebo_perm"; then
  echo "ERROR: gazebo/relay is not allowed to publish rt/cmd_vel" >&2
  exit 1
fi

echo "SROS2 command policy checks passed:"
echo "  controller: can publish /cmd_vel_in"
echo "  attacker: cannot publish /cmd_vel_in"
echo "  gazebo: can bridge/relay command topics"
