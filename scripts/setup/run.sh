#!/bin/bash
# Set log directory
# Get the parent directory of the script's directory (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"

# Launch Gazebo empty world (flat plane) with project custom bridge, redirect output to test.log
cd "$PROJECT_ROOT"
echo "Launching Gazebo empty world (flat plane) with custom bridge..."
ros2 launch "$PROJECT_ROOT/launch/turtlebot3_empty_world_custom_bridge.launch.py" > test.log 2>&1
