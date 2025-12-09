#!/bin/bash
# TurtleBot3 Teleop quick diagnostic script

echo "=========================================="
echo "TurtleBot3 Teleop Diagnostic Tool"
echo "=========================================="
echo ""

echo "=== 1. ROS2 Environment Check ==="
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS_DISTRO not set!"
    echo "   Please run: source /opt/ros/humble/setup.bash (or jazzy)"
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "❌ TURTLEBOT3_MODEL not set!"
    echo "   Please run: export TURTLEBOT3_MODEL=burger"
else
    echo "✅ TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
fi
echo ""

echo "=== 2. Package Check ==="
if ros2 pkg list | grep -q turtlebot3_teleop; then
    echo "✅ turtlebot3_teleop package is installed"
else
    echo "❌ turtlebot3_teleop package not found!"
    echo "   Please run: sudo apt install ros-$ROS_DISTRO-turtlebot3-teleop"
fi
echo ""

echo "=== 3. Topic Check ==="
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ /cmd_vel topic exists"
    echo "   Topic info:"
    ros2 topic info /cmd_vel 2>/dev/null | head -3
else
    echo "❌ /cmd_vel topic does not exist!"
    echo "   Please ensure Gazebo simulation is running (Step 10)"
fi
echo ""

echo "=== 4. Node Check ==="
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "⚠️  No running nodes detected"
    echo "   Please ensure Gazebo simulation is running (Step 10)"
else
    echo "✅ Detected the following nodes:"
    echo "$NODES" | head -5
fi
echo ""

echo "=== 5. Test Publishing Command ==="
echo "Attempting to publish a test command (robot should briefly move forward)..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Command published successfully! If robot moved, interface is working."
else
    echo "❌ Command publish failed! Please check if topic exists."
fi
echo ""

echo "=== 6. Terminal Mode Check ==="
if [ -t 0 ]; then
    echo "✅ Terminal supports interactive input"
    TERM_MODE=$(stty -g 2>/dev/null)
    if [ $? -eq 0 ]; then
        echo "   Current terminal mode: $TERM_MODE"
    fi
else
    echo "⚠️  Current terminal may not be interactive"
    echo "   If connected via SSH, suggest running teleop_keyboard in VM local terminal"
fi
echo ""

echo "=========================================="
echo "Diagnosis complete!"
echo ""
echo "If all checks pass but keyboard control still doesn't work,"
echo "it may be a terminal input issue. Suggestions:"
echo "1. Run teleop_keyboard in VM local terminal (not SSH)"
echo "2. Or use ros2 topic pub command for direct control (see troubleshooting guide)"
echo "=========================================="
