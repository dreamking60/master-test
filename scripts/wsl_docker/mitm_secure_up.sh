#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/mitm_controller/permissions.p7s" ] || \
   [ ! -f "$PROJECT_ROOT/deployment/wsl_docker/keys/enclaves/mitm_robot/permissions.p7s" ]; then
  echo "ERROR: SROS2 MITM permissions not found."
  echo "Run: sudo ./scripts/wsl_docker/init_sros2_docker.sh"
  exit 1
fi

cd "$PROJECT_ROOT"
sudo env \
  ROS_DOMAIN_ID=0 \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ROS_LOCALHOST_ONLY=0 \
  ROS_SECURITY_ENABLE=true \
  ROS_SECURITY_STRATEGY=Enforce \
  "$SCRIPT_DIR/mitm_up.sh"
