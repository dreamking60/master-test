#!/usr/bin/env bash
# Initialize SROS2 keys for the WSL+Docker environment
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KEYS_DIR="$PROJECT_ROOT/deployment/wsl_docker/keys"
POLICY_FILE="$PROJECT_ROOT/config/sros2_wsl_docker_policy.xml"

echo "Creating keys directory: $KEYS_DIR"
mkdir -p "$KEYS_DIR"

if [ ! -f "$POLICY_FILE" ]; then
  echo "ERROR: SROS2 policy file not found: $POLICY_FILE" >&2
  exit 1
fi

echo "Generating SROS2 keystore and enclaves..."

# Define nodes/enclaves
ENCLAVES=("controller" "attacker" "gazebo" "mitm_controller" "mitm_robot")

# Prefer host ROS2 when available. Fall back to Docker for machines where only
# the container image has ros2 security installed.
DOCKER_IMAGE="tb3-ros2-node:jazzy"
USE_DOCKER=0

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1 || ! ros2 security --help >/dev/null 2>&1; then
  USE_DOCKER=1
  if [[ "$(sudo docker images -q $DOCKER_IMAGE 2> /dev/null)" == "" ]]; then
    echo "ERROR: Docker image $DOCKER_IMAGE not found and host ros2 security is unavailable."
    echo "Please run: sudo ./scripts/wsl_docker/up.sh first to build the image."
    exit 1
  fi
fi

run_in_docker() {
    sudo docker run --rm \
        -v "$PROJECT_ROOT:/workspace/project" \
        -w "/workspace/project/deployment/wsl_docker/keys" \
        $DOCKER_IMAGE \
        bash -lc "$1"
}

run_security_cmd() {
  local command="$1"
  if [ "$USE_DOCKER" -eq 1 ]; then
    run_in_docker "$command"
  else
    (cd "$KEYS_DIR" && bash -lc "$command")
  fi
}

echo "[1/3] Creating Keystore..."
run_security_cmd "ros2 security create_keystore . || echo 'Keystore already exists, skipping...'"

echo "[2/3] Creating Enclaves and policy-based permissions..."
if [ "$USE_DOCKER" -eq 1 ]; then
  run_security_cmd "ROS_DOMAIN_ID=0 ros2 security generate_artifacts -k . -e /controller /attacker /gazebo /mitm_controller /mitm_robot -p /workspace/project/config/sros2_wsl_docker_policy.xml"
else
  run_security_cmd "ROS_DOMAIN_ID=0 ros2 security generate_artifacts -k . -e /controller /attacker /gazebo /mitm_controller /mitm_robot -p '$POLICY_FILE'"
fi

echo "Generated permissions summary:"
for name in "${ENCLAVES[@]}"; do
    echo "  - /$name"
    sed -n '1,120p' "$KEYS_DIR/enclaves/$name/permissions.xml" | grep -E '<grant|<topic>|<default>' || true
done

echo "[3/3] Setting permissions (local)..."
if [ "$USE_DOCKER" -eq 1 ]; then
  sudo chown -R $USER:$USER "$KEYS_DIR"
fi

echo ""
echo "✅ SROS2 initialization complete!"
echo "Keys are located in: $KEYS_DIR"
echo ""
echo "Next Steps:"
echo "1. Restart containers with security enabled:"
echo "   sudo ./scripts/wsl_docker/secure_up.sh"
echo ""
echo "2. Run the secure controller:"
echo "   sudo ./scripts/wsl_docker/run_controller.sh"
