#!/bin/bash
# Auto-generated SROS2 environment for gazebo_sim
# Source this file: source setup_gazebo_sim_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE_DIR="$SCRIPT_DIR/keys/gazebo_sim"

# Ensure enclaves directory exists (required by SROS2)
if [ ! -d "$NODE_DIR/enclaves" ]; then
    mkdir -p "$NODE_DIR/enclaves"
    # Try to create keystore and enclave using ros2 security
    if command -v ros2 &> /dev/null; then
        ros2 security create_keystore "$NODE_DIR" 2>/dev/null || true
        ros2 security create_enclave "$NODE_DIR" "/" 2>/dev/null || true
    fi
fi

export ROS_SECURITY_KEYSTORE="$NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo "SROS2 security enabled for: gazebo_sim"
echo "  Keystore: $ROS_SECURITY_KEYSTORE"
echo "  Security: Enabled"
echo "  Strategy: Enforce"
