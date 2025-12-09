#!/bin/bash
# Launch Gazebo with empty world (flat plane) for better visualization
# This is ideal for observing robot movement patterns and attack effects

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch Gazebo with empty world
cd "$PROJECT_ROOT"
echo "Launching Gazebo with empty world (flat plane)..."
echo "This provides a clear view of robot movement patterns."
echo ""

ros2 launch turtlebot3_gazebo empty_world.launch.py > test.log 2>&1

