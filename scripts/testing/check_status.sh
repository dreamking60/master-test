#!/bin/bash
# ROS2 system status check script

echo "=========================================="
echo "ROS2 System Status Check"
echo "=========================================="
echo ""

# Check ROS2 environment
echo "=== 1. ROS2 Environment ==="
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 environment not loaded"
    echo "   Please run: source /opt/ros/humble/setup.bash (or jazzy)"
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "⚠️  TURTLEBOT3_MODEL not set"
    echo "   Suggested: export TURTLEBOT3_MODEL=burger"
else
    echo "✅ TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
fi
echo ""

# Check running nodes
echo "=== 2. Running Nodes ==="
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "⚠️  No running nodes detected"
    echo "   If Gazebo is running, may need to wait a few seconds"
else
    echo "✅ Detected the following nodes:"
    echo "$NODES"
fi
echo ""

# Check topics
echo "=== 3. Available Topics ==="
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
    echo "⚠️  No topics detected"
    echo "   Please ensure Gazebo simulation is running"
else
    echo "✅ Detected the following topics:"
    echo "$TOPICS" | head -10
    if echo "$TOPICS" | grep -q cmd_vel; then
        echo ""
        echo "✅ /cmd_vel topic exists (can control robot)"
    fi
fi
echo ""

# Check processes
echo "=== 4. Related Processes ==="
if pgrep -f "gazebo" > /dev/null; then
    echo "✅ Gazebo process is running"
else
    echo "⚠️  Gazebo process not detected"
fi

if pgrep -f "turtlebot3" > /dev/null; then
    echo "✅ TurtleBot3 related processes are running"
else
    echo "⚠️  TurtleBot3 related processes not detected"
fi
echo ""

# Check logs
echo "=== 5. Log Files ==="
LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -d "$LOG_DIR" ] && [ "$(ls -A $LOG_DIR 2>/dev/null)" ]; then
    LOG_COUNT=$(find "$LOG_DIR" -name "*.log" 2>/dev/null | wc -l)
    echo "✅ Found $LOG_COUNT log file(s) in $LOG_DIR"
    LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -n "$LATEST_LOG" ]; then
        echo "   Latest log: $LATEST_LOG"
    fi
else
    echo "⚠️  Log directory is empty or does not exist"
fi

if [ -f "./test.log" ]; then
    LOG_SIZE=$(du -h "./test.log" | cut -f1)
    echo "✅ test.log exists ($LOG_SIZE)"
else
    echo "⚠️  test.log does not exist"
fi
echo ""

# Summary
echo "=========================================="
echo "Status Summary"
echo "=========================================="
if [ -n "$ROS_DISTRO" ] && [ -n "$NODES" ] && echo "$TOPICS" | grep -q cmd_vel; then
    echo "✅ System appears to be running normally!"
    echo ""
    echo "Next steps:"
    echo "1. Try running: ros2 run turtlebot3_teleop teleop_keyboard"
    echo "2. Or use: python3 test_control.py forward 0.2 2.0"
else
    echo "⚠️  System may not be fully started"
    echo ""
    echo "Suggestions:"
    echo "1. Ensure you've run: ./run.sh (start Gazebo)"
    echo "2. Wait a few seconds for system to fully start"
    echo "3. Run this script again to check status"
fi
echo "=========================================="
