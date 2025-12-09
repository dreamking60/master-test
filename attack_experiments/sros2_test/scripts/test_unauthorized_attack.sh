#!/bin/bash
# Test unauthorized attack attempt (should fail with SROS2)

echo "=========================================="
echo "SROS2 Unauthorized Attack Test"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"

echo "This test attempts to attack a secured ROS2 system."
echo "The attack should FAIL if SROS2 is properly configured."
echo ""

# Check if target has security enabled
read -p "Is the target machine running with SROS2 enabled? [y/N]: " target_secured

if [ "$target_secured" != "y" ]; then
    echo ""
    echo "⚠️  Target is not secured - attack will succeed"
    echo "   This test is for secured systems only"
    exit 1
fi

echo ""
echo "Attempting attack WITHOUT proper credentials..."
echo ""

# Try to attack WITHOUT security credentials
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY

echo "Attack configuration:"
echo "  ROS_SECURITY_KEYSTORE: (not set)"
echo "  ROS_SECURITY_ENABLE: (not set)"
echo "  ROS_SECURITY_STRATEGY: (not set)"
echo ""

echo "Attempting to discover nodes..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 3

nodes=$(ros2 node list 2>/dev/null)
topics=$(ros2 topic list 2>/dev/null)

if [ -z "$nodes" ] && [ -z "$topics" ]; then
    echo "✅ SUCCESS: Attack blocked!"
    echo "   Cannot discover nodes without proper credentials"
    echo "   SROS2 security is working correctly"
elif echo "$topics" | grep -q cmd_vel; then
    echo "❌ FAIL: Attack succeeded!"
    echo "   Can discover /cmd_vel topic without credentials"
    echo "   SROS2 security is NOT working properly"
    echo ""
    echo "Possible reasons:"
    echo "  - Target not using SROS2"
    echo "  - Security policy allows unauthenticated access"
    echo "  - Same network, different security domain"
else
    echo "⚠️  Partial discovery"
    echo "   Some nodes/topics visible but /cmd_vel not accessible"
fi

echo ""
echo "Attempting to publish to /cmd_vel..."
echo ""

# Try to publish (should fail)
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
     twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    --once 2>&1 | head -5

if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo ""
    echo "❌ FAIL: Can publish without credentials!"
    echo "   SROS2 security is NOT working"
else
    echo ""
    echo "✅ SUCCESS: Cannot publish without credentials!"
    echo "   SROS2 security is working correctly"
fi

echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""
echo "If attacks were blocked:"
echo "  ✅ SROS2 security is effective"
echo "  ✅ Unauthorized nodes cannot access system"
echo ""
echo "If attacks succeeded:"
echo "  ❌ SROS2 security is not properly configured"
echo "  ❌ Check security policies and certificates"
echo ""

