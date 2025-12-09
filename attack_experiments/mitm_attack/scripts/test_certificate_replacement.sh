#!/bin/bash
# Test certificate replacement attack
# Attempts to use attacker's certificate to connect to secured ROS2 system

echo "=========================================="
echo "Certificate Replacement Attack Test"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ATTACKER_NODE_DIR="$MITM_DIR/attacker_keys/mitm_attacker"

# Check if setup is done
if [ ! -f "$ATTACKER_NODE_DIR/cert.pem" ]; then
    echo "❌ Error: Attacker certificates not found"
    echo "   Please run: ./setup_mitm_environment.sh first"
    exit 1
fi

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This test attempts to use attacker's certificate (from different CA)"
echo "to connect to a secured ROS2 system."
echo ""
echo "Expected result: Should FAIL (certificate validation should reject)"
echo ""

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Test cancelled"
    exit 0
fi

echo ""
echo "Step 1: Setting attacker's certificate..."
echo ""

# Use attacker's certificate (different CA)
export ROS_SECURITY_KEYSTORE="$ATTACKER_NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo "ROS_SECURITY_KEYSTORE: $ROS_SECURITY_KEYSTORE"
echo "Using attacker's CA (different from target system)"
echo ""

echo "Step 2: Attempting to discover nodes..."
echo ""

# Restart daemon
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
sleep 3

# Try to discover nodes
echo "Checking if we can discover target nodes..."
NODES=$(ros2 node list 2>/dev/null)
TOPICS=$(ros2 topic list 2>/dev/null)

echo ""
echo "=========================================="
echo "Test Results"
echo "=========================================="
echo ""

if [ -z "$NODES" ]; then
    echo "✅ SUCCESS (for security): No nodes discovered"
    echo "   Certificate validation REJECTED attacker's certificate"
    echo "   This is the CORRECT behavior - MITM attack FAILED"
    echo ""
    echo "Conclusion: SROS2 is properly defending against MITM"
else
    echo "⚠️  WARNING: Nodes discovered with attacker's certificate!"
    echo "   Discovered nodes:"
    echo "$NODES" | head -5
    echo ""
    echo "This suggests:"
    echo "  - Certificate validation may not be working correctly"
    echo "  - Target system may not be using SROS2 properly"
    echo "  - CA verification may be misconfigured"
fi

if [ -z "$TOPICS" ]; then
    echo "✅ SUCCESS (for security): No topics discovered"
    echo "   Cannot access /cmd_vel topic"
else
    if echo "$TOPICS" | grep -q cmd_vel; then
        echo "⚠️  WARNING: /cmd_vel topic discovered!"
        echo "   Attacker can potentially publish to /cmd_vel"
        echo "   This indicates a security vulnerability"
    else
        echo "✅ /cmd_vel topic not accessible"
    fi
fi

echo ""
echo "Step 3: Attempting to publish to /cmd_vel..."
echo ""

# Try to publish a test message
if ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    2>&1 | grep -q "error\|failed\|reject"; then
    echo "✅ SUCCESS (for security): Message publish REJECTED"
    echo "   Certificate validation blocked the attack"
else
    if ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
        2>&1 | grep -q "publishing"; then
        echo "⚠️  WARNING: Message publish SUCCEEDED!"
        echo "   This indicates a security vulnerability"
        echo "   MITM attack may have succeeded"
    else
        echo "✅ Message publish failed (expected)"
    fi
fi

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - If all checks show ✅: SROS2 is defending correctly"
echo "  - If any check shows ⚠️: Security configuration may be incomplete"
echo ""

# Clean up
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY

