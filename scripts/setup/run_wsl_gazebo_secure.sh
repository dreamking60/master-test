#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

set +u
source "$SCRIPT_DIR/source_wsl_ros_env.sh"
set -u

export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE="$PROJECT_ROOT/deployment/wsl_docker/keys"
export ROS_SECURITY_ENCLAVE_OVERRIDE=/gazebo

if [ ! -f "$ROS_SECURITY_KEYSTORE/enclaves/gazebo/permissions.p7s" ]; then
  echo "ERROR: SROS2 gazebo permissions not found."
  echo "Run: sudo ./scripts/wsl_docker/init_sros2_docker.sh"
  exit 1
fi

echo "SROS2 secure Gazebo env:"
echo "  ROS_SECURITY_ENABLE=$ROS_SECURITY_ENABLE"
echo "  ROS_SECURITY_STRATEGY=$ROS_SECURITY_STRATEGY"
echo "  ROS_SECURITY_KEYSTORE=$ROS_SECURITY_KEYSTORE"
echo "  ROS_SECURITY_ENCLAVE_OVERRIDE=$ROS_SECURITY_ENCLAVE_OVERRIDE"

exec "$SCRIPT_DIR/run.sh"
