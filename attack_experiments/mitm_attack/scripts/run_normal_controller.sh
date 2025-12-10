#!/bin/bash
# One-click script to run normal controller for MITM attack test

echo "=========================================="
echo "Normal Controller Launcher"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Clear SROS2 for MITM test (we want unencrypted communication)
echo "Clearing SROS2 configuration for MITM test..."
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY
echo "✅ Running without SROS2 (unencrypted)"

echo "Step 1: Checking Gazebo..."
if ! pgrep -f "gz sim" > /dev/null && ! pgrep -f "gazebo" > /dev/null; then
    echo "❌ Gazebo is not running!"
    echo ""
    echo "Please start Gazebo first:"
    echo "  cd $PROJECT_ROOT"
    echo "  ./scripts/setup/restart_gazebo.sh"
    echo ""
    read -p "Start Gazebo now? [y/N]: " start_gazebo
    if [ "$start_gazebo" = "y" ]; then
        cd "$PROJECT_ROOT"
        ./scripts/setup/restart_gazebo.sh &
        echo "Waiting for Gazebo to start..."
        sleep 10
    else
        exit 1
    fi
else
    echo "✅ Gazebo is running"
fi

echo ""
echo "Step 2: Checking /cmd_vel topic..."
sleep 2
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ /cmd_vel topic exists"
else
    echo "⚠️  /cmd_vel topic not found"
    echo "   Waiting a bit more for Gazebo to fully start..."
    sleep 5
    if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
        echo "✅ /cmd_vel topic found"
    else
        echo "❌ /cmd_vel topic still not found"
        echo "   Please check Gazebo is running correctly"
        exit 1
    fi
fi

echo ""
echo "Step 3: Starting normal controller..."
echo ""

# Get path to normal_controller.py
CONTROLLER_SCRIPT="$PROJECT_ROOT/attack_experiments/scripts/normal_controller.py"

if [ ! -f "$CONTROLLER_SCRIPT" ]; then
    echo "❌ normal_controller.py not found at $CONTROLLER_SCRIPT"
    exit 1
fi

echo "Controller will run for 60 seconds (continuous forward pattern)"
echo "You should see the robot moving forward in Gazebo"
echo ""

# Test if messages are being sent
echo "Testing message publishing..."
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    --rate 10 > /dev/null 2>&1 &

sleep 2
echo "✅ Test command sent"
echo ""

read -p "Did you see the robot move? [y/N]: " robot_moved
if [ "$robot_moved" != "y" ]; then
    echo ""
    echo "⚠️  Robot didn't move. Possible issues:"
    echo "  1. Check Gazebo window - is robot visible?"
    echo "  2. Run diagnostic: ./test_robot_movement.sh"
    echo "  3. Check if robot is stuck or collided"
    echo ""
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
        exit 1
    fi
fi

echo ""
echo "Starting normal controller..."
echo "Press Ctrl+C to stop early"
echo ""

# Run controller
python3 "$CONTROLLER_SCRIPT" \
    --pattern continuous \
    --frequency 10 \
    --duration 60 \
    --speed 0.2

EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "Controller finished"
else
    echo "Controller exited with error (code: $EXIT_CODE)"
fi
echo "=========================================="

