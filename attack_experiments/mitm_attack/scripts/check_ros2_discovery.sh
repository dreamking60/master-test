#!/bin/bash
# Diagnostic script to check ROS2 discovery in MITM setup

echo "=========================================="
echo "ROS2 Discovery Diagnostic Tool"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

# Load configuration
ROBOT_IP=""
CONTROLLER_IP=""
ROUTER_IP=""
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

if [ -f "$CONFIG_DIR/robot_config.txt" ]; then
    source "$CONFIG_DIR/robot_config.txt"
fi

if [ -f "$CONFIG_DIR/controller_config.txt" ]; then
    source "$CONFIG_DIR/controller_config.txt"
fi

if [ -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    source "$CONFIG_DIR/mitm_router_config.txt"
fi

echo "Configuration:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  Robot IP: ${ROBOT_IP:-not configured}"
echo "  Controller IP: ${CONTROLLER_IP:-not configured}"
echo "  Router IP: ${ROUTER_IP:-not configured}"
echo ""

# Check network connectivity
echo "Step 1: Network Connectivity"
echo "------------------------------"
if [ -n "$ROBOT_IP" ]; then
    echo -n "Pinging robot ($ROBOT_IP): "
    if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
        echo "✅ OK"
    else
        echo "❌ FAILED"
        echo "   Cannot reach robot machine"
    fi
fi

if [ -n "$ROUTER_IP" ]; then
    echo -n "Pinging router ($ROUTER_IP): "
    if ping -c 1 -W 2 "$ROUTER_IP" > /dev/null 2>&1; then
        echo "✅ OK"
    else
        echo "❌ FAILED"
        echo "   Cannot reach router"
    fi
fi

# Check gateway
CURRENT_GATEWAY=$(ip route | grep default | awk '{print $3}')
echo "Current gateway: ${CURRENT_GATEWAY:-none}"
if [ -n "$ROUTER_IP" ] && [ "$CURRENT_GATEWAY" = "$ROUTER_IP" ]; then
    echo "✅ Gateway correctly set to router"
else
    echo "⚠️  Gateway not set to router"
    if [ -n "$ROUTER_IP" ]; then
        echo "   Expected: $ROUTER_IP"
    fi
fi
echo ""

# Check ROS2 daemon
echo "Step 2: ROS2 Daemon"
echo "------------------------------"
if ros2 daemon status > /dev/null 2>&1; then
    echo "✅ ROS2 daemon is running"
else
    echo "⚠️  ROS2 daemon not running, starting..."
    ros2 daemon start
    sleep 2
fi
echo ""

# Check DDS configuration
echo "Step 3: DDS Configuration"
echo "------------------------------"
if [ -n "$CYCLONEDDS_URI" ]; then
    echo "✅ CYCLONEDDS_URI: $CYCLONEDDS_URI"
    if [ -f "$CONFIG_DIR/dds_controller_config.xml" ]; then
        echo "✅ DDS config file exists"
    fi
else
    echo "⚠️  No DDS configuration (using default multicast)"
    if [ -n "$ROBOT_IP" ]; then
        echo "   Consider configuring DDS peer discovery for: $ROBOT_IP"
    fi
fi
echo ""

# Check ROS2 discovery
echo "Step 4: ROS2 Discovery"
echo "------------------------------"
echo "Waiting 5 seconds for discovery..."
sleep 5

NODES=$(ros2 node list 2>/dev/null)
TOPICS=$(ros2 topic list 2>/dev/null)

if [ -n "$NODES" ]; then
    echo "✅ Discovered nodes:"
    echo "$NODES" | sed 's/^/  /'
else
    echo "❌ No nodes discovered"
fi
echo ""

if [ -n "$TOPICS" ]; then
    echo "✅ Discovered topics:"
    echo "$TOPICS" | sed 's/^/  /'
    
    if echo "$TOPICS" | grep -q cmd_vel; then
        echo ""
        echo "✅ /cmd_vel topic found!"
    else
        echo ""
        echo "❌ /cmd_vel topic not found"
    fi
else
    echo "❌ No topics discovered"
fi
echo ""

# Check DDS ports
echo "Step 5: DDS Ports (7400-7500)"
echo "------------------------------"
if command -v netstat > /dev/null 2>&1; then
    DDS_PORTS=$(netstat -uln 2>/dev/null | grep -E ':(74[0-9]{2}|7500)' | wc -l)
    echo "DDS ports in use: $DDS_PORTS"
elif command -v ss > /dev/null 2>&1; then
    DDS_PORTS=$(ss -uln 2>/dev/null | grep -E ':(74[0-9]{2}|7500)' | wc -l)
    echo "DDS ports in use: $DDS_PORTS"
fi
echo ""

# Summary
echo "=========================================="
echo "Summary"
echo "=========================================="
if echo "$TOPICS" | grep -q cmd_vel; then
    echo "✅ ROS2 discovery is working!"
    echo "   You should be able to run the controller now"
else
    echo "❌ ROS2 discovery failed"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Ensure Gazebo is running on robot machine"
    echo "  2. Check ROS_DOMAIN_ID matches on both machines (current: $ROS_DOMAIN_ID)"
    echo "  3. Verify network routing through MITM router"
    echo "  4. Check firewall on DDS ports (7400-7500)"
    echo "  5. Try running: ros2 daemon stop && ros2 daemon start"
    if [ -n "$ROBOT_IP" ]; then
        echo "  6. Test connectivity: ping $ROBOT_IP"
    fi
fi
echo ""

