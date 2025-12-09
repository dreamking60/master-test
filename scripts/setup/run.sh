#!/bin/bash
# Set log directory
# Get the parent directory of the script's directory (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"

# Launch Gazebo simulation, redirect output to test.log in project root
cd "$PROJECT_ROOT"
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > test.log 2>&1
