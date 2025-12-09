#!/bin/bash
# Test ROS_DOMAIN_ID isolation defense mechanism
# This test verifies that different ROS_DOMAIN_IDs prevent communication

echo "=========================================="
echo "Defense Test: ROS_DOMAIN_ID Isolation"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This test demonstrates that ROS_DOMAIN_ID provides basic isolation."
echo ""
echo "Test scenario:"
echo "  1. Target machine uses ROS_DOMAIN_ID=0"
echo "  2. Attacker uses ROS_DOMAIN_ID=42 (different)"
echo "  3. Attacker should NOT be able to discover or attack target"
echo ""

read -p "Press Enter to start test..."

# Get target IP
read -p "Enter target machine IP address: " target_ip

echo ""
echo "=========================================="
echo "Test 1: Same Domain ID (Should Work)"
echo "=========================================="
echo ""

export ROS_DOMAIN_ID=0
echo "Attacker ROS_DOMAIN_ID: 0 (same as target)"
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 3

echo "Checking if target is discoverable..."
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ SUCCESS: Target discovered with same domain ID"
    echo "   This shows the attack would work without isolation"
else
    echo "⚠️  Target not found (may be normal if Gazebo not running)"
fi
echo ""

echo "=========================================="
echo "Test 2: Different Domain ID (Should Fail)"
echo "=========================================="
echo ""

export ROS_DOMAIN_ID=42
echo "Attacker ROS_DOMAIN_ID: 42 (different from target)"
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 3

echo "Checking if target is discoverable..."
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "❌ FAIL: Target still discoverable with different domain ID"
    echo "   This means ROS_DOMAIN_ID isolation is NOT working"
    echo "   Possible reasons:"
    echo "   - DDS routing configured to bypass isolation"
    echo "   - Both machines using same multicast address"
else
    echo "✅ SUCCESS: Target NOT discoverable with different domain ID"
    echo "   ROS_DOMAIN_ID isolation is working!"
    echo "   Attacker cannot discover or attack target"
fi
echo ""

echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""
echo "ROS_DOMAIN_ID isolation:"
echo "  - Provides basic network isolation"
echo "  - Easy to implement (just set environment variable)"
echo "  - NOT secure (attacker can guess or brute force)"
echo "  - Recommended: Use random, non-default domain IDs"
echo ""
echo "Defense effectiveness: ⭐⭐ (2/5)"
echo "  - Works against casual attackers"
echo "  - Does NOT work against determined attackers"
echo "  - Should be combined with other defenses"
echo ""

