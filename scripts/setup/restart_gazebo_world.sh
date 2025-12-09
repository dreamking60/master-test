#!/bin/bash
# One-click Gazebo restart script (with turtlebot3_world)
# Stops all Gazebo processes and restarts with turtlebot3_world

echo "=========================================="
echo "Gazebo Restart (turtlebot3_world)"
echo "=========================================="
echo ""

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Step 1: Stop all Gazebo processes
echo "Step 1: Stopping existing Gazebo processes..."
echo ""

# Kill Gazebo processes
pkill -f "gazebo" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "gz server" 2>/dev/null
pkill -f "gz client" 2>/dev/null
pkill -f "empty_world.launch.py" 2>/dev/null
pkill -f "turtlebot3_world.launch.py" 2>/dev/null

# Wait for processes to terminate
sleep 2

# Force kill if still running
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "gz" 2>/dev/null

echo "✅ Gazebo processes stopped"
echo ""

# Step 2: Restart ROS2 daemon
echo "Step 2: Restarting ROS2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
echo "✅ ROS2 daemon restarted"
echo ""

# Step 3: Setup log directory
echo "Step 3: Setting up log directory..."
export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"
echo "✅ Log directory ready: $ROS_LOG_DIR"
echo ""

# Step 4: Restart Gazebo
echo "=========================================="
echo "Step 4: Starting Gazebo with turtlebot3_world..."
echo "=========================================="
echo ""

cd "$PROJECT_ROOT"
echo "Launching Gazebo with turtlebot3_world..."
echo ""
echo "Press Ctrl+C to stop Gazebo"
echo ""

# Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > test.log 2>&1

