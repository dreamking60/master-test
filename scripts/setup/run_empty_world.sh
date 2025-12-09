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

# IMPORTANT: Fix SROS2 security if configured but incomplete
# Create missing enclaves directory instead of disabling security
if [ -n "$ROS_SECURITY_KEYSTORE" ] && [ ! -d "$ROS_SECURITY_KEYSTORE/enclaves" ]; then
    echo "Warning: SROS2 security configured but enclaves directory missing."
    echo "Creating enclaves directory..."
    mkdir -p "$ROS_SECURITY_KEYSTORE/enclaves"
    
    # Try to create keystore and enclave using ros2 security
    if command -v ros2 &> /dev/null; then
        ros2 security create_keystore "$ROS_SECURITY_KEYSTORE" 2>/dev/null || true
        ros2 security create_enclave "$ROS_SECURITY_KEYSTORE" "/" 2>/dev/null || true
    fi
    
    if [ -d "$ROS_SECURITY_KEYSTORE/enclaves" ]; then
        echo "✅ Enclaves directory created - security mode enabled"
    else
        echo "⚠️  Failed to create enclaves, but continuing..."
    fi
    echo ""
fi

# Launch Gazebo with empty world
cd "$PROJECT_ROOT"
echo "Launching Gazebo with empty world (flat plane)..."
echo "This provides a clear view of robot movement patterns."
echo ""

ros2 launch turtlebot3_gazebo empty_world.launch.py > test.log 2>&1

