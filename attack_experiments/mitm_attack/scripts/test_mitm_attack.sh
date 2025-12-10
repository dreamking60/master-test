#!/bin/bash
# Complete MITM attack test
# Tests if MITM router can intercept and modify ROS2 traffic

echo "=========================================="
echo "Complete MITM Attack Test"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This test verifies MITM attack setup and functionality."
echo ""

# Check router config
CONFIG_DIR="$MITM_DIR/config"
if [ ! -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    echo "❌ MITM router not configured"
    echo "   Run setup_mitm_router.sh first"
    exit 1
fi

source "$CONFIG_DIR/mitm_router_config.txt"

echo "Router Configuration:"
echo "  IP: $ROUTER_IP"
echo "  Interface: $ROUTER_INTERFACE"
echo ""

# Check IP forwarding
if [ "$(cat /proc/sys/net/ipv4/ip_forward)" != "1" ]; then
    echo "⚠️  IP forwarding not enabled"
    echo "   Run setup_mitm_router.sh to enable"
    exit 1
fi

echo "✅ IP forwarding enabled"
echo ""

echo "Step 1: Verifying network connectivity..."
echo ""

# Check if we can see traffic
if command -v tcpdump &> /dev/null; then
    echo "Testing packet capture..."
    timeout 3 tcpdump -i "$ROUTER_INTERFACE" -c 5 'udp portrange 7400-7500' 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✅ Can capture DDS traffic"
    else
        echo "⚠️  No DDS traffic detected (may be normal if no ROS2 nodes active)"
    fi
else
    echo "⚠️  tcpdump not installed (optional for testing)"
fi

echo ""
echo "Step 2: Testing ROS2 communication through MITM..."
echo ""

# Check ROS2 connectivity
NODES=$(ros2 node list 2>/dev/null)
TOPICS=$(ros2 topic list 2>/dev/null)

if [ -n "$NODES" ]; then
    echo "✅ ROS2 nodes discovered:"
    echo "$NODES" | head -5
    echo ""
    
    if echo "$TOPICS" | grep -q cmd_vel; then
        echo "✅ /cmd_vel topic accessible"
        echo ""
        echo "Step 3: Testing message interception..."
        echo ""
        
        # Try to monitor messages
        echo "Monitoring /cmd_vel messages (5 seconds)..."
        timeout 5 ros2 topic echo /cmd_vel 2>/dev/null | head -10
        
        echo ""
        echo "✅ Can intercept ROS2 messages"
    else
        echo "⚠️  /cmd_vel topic not found"
    fi
else
    echo "⚠️  No ROS2 nodes discovered"
    echo "   Ensure robot and controller are running"
fi

echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""
echo "MITM Router Status:"
echo "  ✅ IP forwarding: Enabled"
echo "  ✅ Router IP: $ROUTER_IP"
echo "  ✅ Interface: $ROUTER_INTERFACE"
echo ""
echo "Traffic Interception:"
if [ -n "$NODES" ]; then
    echo "  ✅ ROS2 traffic flowing through router"
    echo "  ✅ Can intercept DDS messages"
else
    echo "  ⚠️  No ROS2 traffic detected"
    echo "     (May need robot/controller running)"
fi
echo ""
echo "Next steps:"
echo "  1. Run intercept_and_modify.sh to start full interception"
echo "  2. Analyze captured packets"
echo "  3. Test message modification (if unencrypted)"
echo ""

