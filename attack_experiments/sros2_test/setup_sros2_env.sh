#!/bin/bash
# SROS2 Environment Setup Script
# Source this file to enable SROS2 security

if [ -z "$1" ]; then
    echo "Usage: source setup_sros2_env.sh <node_name>"
    echo ""
    echo "Available nodes:"
    ls -d keys/*/ 2>/dev/null | sed 's|keys/||' | sed 's|/||' | sed 's/^/  - /'
    return 1
fi

NODE_NAME="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/keys"
NODE_DIR="$KEYS_DIR/$NODE_NAME"

if [ ! -d "$NODE_DIR" ]; then
    echo "Error: Node '$NODE_NAME' not found"
    echo "Available nodes:"
    ls -d keys/*/ 2>/dev/null | sed 's|keys/||' | sed 's|/||' | sed 's/^/  - /'
    return 1
fi

if [ ! -f "$NODE_DIR/cert.pem" ]; then
    echo "Error: Certificate not found for '$NODE_NAME'"
    return 1
fi

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

echo "SROS2 security enabled for: $NODE_NAME"
echo "  Keystore: $ROS_SECURITY_KEYSTORE"
echo "  Security: Enabled"
echo "  Strategy: Enforce"
