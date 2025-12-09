#!/bin/bash
# Launch Gazebo with empty world WITHOUT SROS2 security
# Use this when you don't need security, or when security config is incomplete

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# IMPORTANT: Clear SROS2 security variables to avoid errors
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY

echo "Launching Gazebo with empty world (no security)..."
echo "SROS2 security variables cleared."
echo ""

# Launch Gazebo with empty world
cd "$PROJECT_ROOT"
ros2 launch turtlebot3_gazebo empty_world.launch.py > test.log 2>&1

