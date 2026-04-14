#!/usr/bin/env bash
# Initialize SROS2 keys for the WSL+Docker environment
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KEYS_DIR="$PROJECT_ROOT/deployment/wsl_docker/keys"

echo "Creating keys directory: $KEYS_DIR"
mkdir -p "$KEYS_DIR"

# We use the controller container to run ros2 security commands
# because ROS2 is only installed in Docker.

echo "Generating SROS2 Keystore and Enclaves using Docker..."

# Define nodes/enclaves
ENCLAVES=("controller" "attacker" "gazebo")

# Run ros2 security commands inside a temporary container
# We use the same image as our nodes
DOCKER_IMAGE="tb3-ros2-node:jazzy"

# Check if image exists, if not, user needs to build it
if [[ "$(sudo docker images -q $DOCKER_IMAGE 2> /dev/null)" == "" ]]; then
  echo "ERROR: Docker image $DOCKER_IMAGE not found."
  echo "Please run: sudo ./scripts/wsl_docker/up.sh first to build the image."
  exit 1
fi

# Function to run cmd in container
run_in_docker() {
    sudo docker run --rm \
        -v "$PROJECT_ROOT:/workspace/project" \
        -w "/workspace/project/deployment/wsl_docker/keys" \
        $DOCKER_IMAGE \
        bash -lc "$1"
}

echo "[1/3] Creating Keystore..."
run_in_docker "ros2 security create_keystore . || echo 'Keystore already exists, skipping...'"

echo "[2/3] Creating Enclaves..."
for name in "${ENCLAVES[@]}"; do
    echo "  - Creating enclave for: /$name"
    run_in_docker "ros2 security create_enclave . /$name || echo 'Enclave /$name already exists or failed to create.'"
done

echo "[3/3] Setting permissions (local)..."
sudo chown -R $USER:$USER "$KEYS_DIR"

echo ""
echo "✅ SROS2 initialization complete!"
echo "Keys are located in: $KEYS_DIR"
echo ""
echo "Next Steps:"
echo "1. Restart containers with security enabled:"
echo "   export ROS_SECURITY_ENABLE=true"
echo "   ./scripts/wsl_docker/up.sh"
echo ""
echo "2. Run the secure controller:"
echo "   ./scripts/wsl_docker/run_controller.sh"
