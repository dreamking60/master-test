#!/bin/bash
# Test if robot can move

echo "=========================================="
echo "Robot Movement Test"
echo "=========================================="
echo ""

# Ensure ROS2 environment is loaded
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

export TURTLEBOT3_MODEL=burger

echo "Step 1: Get initial position..."
INITIAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)
if [ -z "$INITIAL_POSE" ]; then
    echo "❌ Cannot get initial position, please ensure Gazebo is running"
    exit 1
fi

INITIAL_X=$(echo "$INITIAL_POSE" | grep "x:" | awk '{print $2}')
INITIAL_Y=$(echo "$INITIAL_POSE" | grep "y:" | awk '{print $2}')

echo "Initial position: x=$INITIAL_X, y=$INITIAL_Y"
echo ""

echo "Step 2: Send forward command (0.2 m/s, duration 3 seconds)..."
# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/test_control.py" forward 0.2 3.0 &
CONTROL_PID=$!

# Wait for command to complete
wait $CONTROL_PID
echo ""

echo "Step 3: Wait 1 second for position to stabilize..."
sleep 1
echo ""

echo "Step 4: Get final position..."
FINAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)
if [ -z "$FINAL_POSE" ]; then
    echo "❌ Cannot get final position"
    exit 1
fi

FINAL_X=$(echo "$FINAL_POSE" | grep "x:" | awk '{print $2}')
FINAL_Y=$(echo "$FINAL_POSE" | grep "y:" | awk '{print $2}')

echo "Final position: x=$FINAL_X, y=$FINAL_Y"
echo ""

echo "Step 5: Calculate movement distance..."
# Use Python for precise calculation
DELTA_X=$(python3 -c "print($FINAL_X - $INITIAL_X)")
DELTA_Y=$(python3 -c "print($FINAL_Y - $INITIAL_Y)")
DISTANCE=$(python3 -c "import math; print(math.sqrt($DELTA_X**2 + $DELTA_Y**2))")

echo "Position change: Δx=$DELTA_X m, Δy=$DELTA_Y m"
echo "Movement distance: $DISTANCE m"
echo ""

echo "=========================================="
echo "Test Results"
echo "=========================================="

# Determine if moved (threshold 0.01 m)
THRESHOLD=0.01
MOVED=$(python3 -c "print(1 if $DISTANCE > $THRESHOLD else 0)")
if [ "$MOVED" = "1" ]; then
    echo "✅ Success! Robot moved $DISTANCE m"
    echo "✅ Control interface /cmd_vel is working"
    echo ""
    echo "Next steps: Continue testing other control commands"
    echo "  - python3 test_control.py left 0.5 2.0"
    echo "  - python3 test_control.py right 0.5 2.0"
    echo "  - python3 test_control.py spin 3.0"
else
    echo "⚠️  Robot seems not to have moved (distance change < $THRESHOLD m)"
    echo ""
    echo "Possible reasons:"
    echo "1. Robot may be stuck or collided"
    echo "2. Control command may not have been sent correctly"
    echo "3. Need to check robot status in Gazebo"
    echo ""
    echo "Suggestions:"
    echo "1. Check Gazebo window, confirm robot is visible"
    echo "2. Check logs: ./view_logs.sh"
    echo "3. Try manually publishing commands to test"
fi
echo "=========================================="
