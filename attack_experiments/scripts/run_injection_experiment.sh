#!/bin/bash
# Injection Attack Experiment Script
# This script runs a realistic attack scenario:
# 1. Normal controller operates the robot
# 2. Attack script attempts to inject and override

echo "=========================================="
echo "Injection Attack Experiment"
echo "=========================================="
echo ""
echo "This experiment demonstrates:"
echo "  1. Normal controller operating robot at regular frequency"
echo "  2. Attack script attempting to inject and override control"
echo ""

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 environment not loaded"
    echo "Loading ROS2 environment..."
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
    if [ -z "$ROS_DISTRO" ]; then
        echo "❌ Failed to load ROS2 environment"
        exit 1
    fi
fi

export TURTLEBOT3_MODEL=burger

echo "✅ ROS2 Environment: $ROS_DISTRO"
echo ""

# Check if Gazebo is running
echo "Checking if Gazebo simulation is running..."
if ! ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "⚠️  Gazebo simulation not detected"
    echo "Please start Gazebo first:"
    echo "  ./scripts/setup/run.sh"
    echo ""
    read -p "Press Enter after starting Gazebo, or Ctrl+C to cancel..."
fi

echo ""
echo "=========================================="
echo "Experiment Configuration"
echo "=========================================="
echo ""

# Get experiment parameters
read -p "Normal controller pattern [square/circle/forward_backward/continuous] (default: square): " pattern
pattern=${pattern:-square}

read -p "Normal controller frequency (Hz) (default: 10): " normal_freq
normal_freq=${normal_freq:-10}

read -p "Normal controller duration (seconds) (default: 60): " normal_duration
normal_duration=${normal_duration:-60}

read -p "Attack type [override/spin/interference/stealth] (default: override): " attack_type
attack_type=${attack_type:-override}

read -p "Attack frequency (Hz) - higher = better override chance (default: 50): " attack_freq
attack_freq=${attack_freq:-50}

read -p "Attack start delay (seconds) - when to start attack (default: 10): " attack_delay
attack_delay=${attack_delay:-10}

read -p "Attack duration (seconds) (default: 20): " attack_duration
attack_duration=${attack_duration:-20}

echo ""
echo "=========================================="
echo "Starting Experiment"
echo "=========================================="
echo ""
echo "Normal Controller:"
echo "  Pattern: $pattern"
echo "  Frequency: $normal_freq Hz"
echo "  Duration: $normal_duration seconds"
echo ""
echo "Attack:"
echo "  Type: $attack_type"
echo "  Frequency: $attack_freq Hz"
echo "  Start delay: $attack_delay seconds"
echo "  Duration: $attack_duration seconds"
echo ""
echo "Press Ctrl+C to stop the experiment"
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Start normal controller in background
echo "[1/2] Starting normal controller..."
python3 "$SCRIPT_DIR/normal_controller.py" \
    --pattern "$pattern" \
    --frequency "$normal_freq" \
    --duration "$normal_duration" \
    --node-name "normal_operator" &
NORMAL_PID=$!

echo "✅ Normal controller started (PID: $NORMAL_PID)"
echo ""

# Wait for attack delay
echo "[2/2] Waiting $attack_delay seconds before starting attack..."
sleep "$attack_delay"

echo "🚨 Starting injection attack..."
echo ""

# Start attack
python3 "$SCRIPT_DIR/injection_attack.py" \
    --attack-type "$attack_type" \
    --frequency "$attack_freq" \
    --duration "$attack_duration" \
    --node-name "injection_attacker"

ATTACK_EXIT=$?

echo ""
echo "=========================================="
echo "Attack Completed"
echo "=========================================="

# Wait for normal controller to finish or kill it
if kill -0 $NORMAL_PID 2>/dev/null; then
    echo "Normal controller still running..."
    read -p "Stop normal controller? [y/N]: " stop_normal
    if [ "$stop_normal" = "y" ] || [ "$stop_normal" = "Y" ]; then
        kill $NORMAL_PID 2>/dev/null
        echo "✅ Normal controller stopped"
    fi
fi

echo ""
echo "Experiment finished!"
echo ""
echo "Observe the robot behavior in Gazebo:"
echo "  - Did the attack successfully override normal control?"
echo "  - How long did it take for the attack to take effect?"
echo "  - What was the robot's behavior during the attack?"

