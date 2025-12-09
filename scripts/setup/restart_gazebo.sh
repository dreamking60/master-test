#!/bin/bash
# One-click Gazebo restart script
# Stops all Gazebo processes and restarts with fresh state

echo "=========================================="
echo "Gazebo Restart"
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

# Step 2: Stop ROS2 daemon (optional, but helps with clean restart)
echo "Step 2: Restarting ROS2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
echo "✅ ROS2 daemon restarted"
echo ""

# Step 3: Clear SROS2 security if needed (optional)
if [ -n "$ROS_SECURITY_KEYSTORE" ] && [ ! -d "$ROS_SECURITY_KEYSTORE/enclaves" ]; then
    echo "Step 3: Fixing SROS2 security configuration..."
    mkdir -p "$ROS_SECURITY_KEYSTORE/enclaves"
    if command -v ros2 &> /dev/null; then
        ros2 security create_keystore "$ROS_SECURITY_KEYSTORE" 2>/dev/null || true
        ros2 security create_enclave "$ROS_SECURITY_KEYSTORE" "/" 2>/dev/null || true
    fi
    echo "✅ SROS2 security fixed"
    echo ""
else
    echo "Step 3: Skipping SROS2 check (not configured or already fixed)"
    echo ""
fi

# Step 4: Setup log directory
echo "Step 4: Setting up log directory..."
export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"
echo "✅ Log directory ready: $ROS_LOG_DIR"
echo ""

# Step 5: Restart Gazebo
echo "=========================================="
echo "Step 5: Starting Gazebo with empty world..."
echo "=========================================="
echo ""

cd "$PROJECT_ROOT"
echo "Launching Gazebo with empty world (flat plane)..."
echo "This provides a clear view of robot movement patterns."
echo ""
echo "Press Ctrl+C to stop Gazebo"
echo ""

# Launch Gazebo
ros2 launch turtlebot3_gazebo empty_world.launch.py > test.log 2>&1

