#!/bin/bash
# Setup script for TARGET machine (the one being attacked)
# Run this on the machine that will run Gazebo and the robot

echo "=========================================="
echo "Target Machine Setup - Network Attack Experiment"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Get current machine IP
CURRENT_IP=$(hostname -I | awk '{print $1}')
echo "Current machine IP: $CURRENT_IP"
echo ""

# Configure ROS_DOMAIN_ID
read -p "Enter ROS_DOMAIN_ID (default 0, press Enter for default): " domain_id
domain_id=${domain_id:-0}
export ROS_DOMAIN_ID=$domain_id
echo "ROS_DOMAIN_ID set to: $domain_id"
echo ""

# Save configuration for later
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
echo "$CURRENT_IP" > "$NETWORK_DIR/target_ip.txt"
echo "$domain_id" > "$NETWORK_DIR/target_domain_id.txt"

echo "=========================================="
echo "Configuration saved!"
echo "=========================================="
echo ""
echo "Target machine information:"
echo "  IP Address: $CURRENT_IP"
echo "  ROS_DOMAIN_ID: $domain_id"
echo ""
echo "Share this information with the attacker machine:"
echo "  IP: $CURRENT_IP"
echo "  Domain ID: $domain_id"
echo ""
echo "=========================================="
echo "Starting Gazebo simulation..."
echo "=========================================="
echo ""
echo "The robot will start and wait for commands."
echo "Press Ctrl+C to stop."
echo ""

# Start Gazebo
cd "$(cd "$SCRIPT_DIR/../../.." && pwd)"
./scripts/setup/run_empty_world.sh

