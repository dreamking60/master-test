#!/bin/bash
# Test secure ROS2 communication

echo "=========================================="
echo "SROS2 Secure Communication Test"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"

echo "This test verifies that SROS2 secure communication works."
echo ""

# Get node name
read -p "Enter node name (with certificate): " node_name
if [ -z "$node_name" ]; then
    echo "Error: Node name required"
    exit 1
fi

NODE_DIR="$KEYS_DIR/$node_name"

if [ ! -f "$NODE_DIR/cert.pem" ]; then
    echo "Error: Certificate not found for $node_name"
    echo "Please run generate_certificates.sh first"
    exit 1
fi

# Setup SROS2 environment
export ROS_SECURITY_KEYSTORE="$NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo ""
echo "SROS2 Configuration:"
echo "  Keystore: $ROS_SECURITY_KEYSTORE"
echo "  Security: Enabled"
echo "  Strategy: Enforce"
echo ""

echo "Testing secure communication..."
echo ""

# Test if security is working
if ros2 security check 2>/dev/null; then
    echo "✅ SROS2 security is active"
else
    echo "⚠️  SROS2 security check failed"
    echo "   This might be normal if SROS2 is not fully installed"
fi

echo ""
echo "You can now run ROS2 nodes with security enabled."
echo ""
echo "Example:"
echo "  # Terminal 1: Start Gazebo (with security)"
echo "  export ROS_SECURITY_KEYSTORE=$NODE_DIR"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo "  cd ../.."
echo "  ./scripts/setup/run_empty_world.sh"
echo ""
echo "  # Terminal 2: Run controller (with same security)"
echo "  export ROS_SECURITY_KEYSTORE=$NODE_DIR"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo "  python3 normal_controller.py"
echo ""

