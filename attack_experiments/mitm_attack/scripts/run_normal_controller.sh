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

# Load MITM configuration
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"
ROBOT_IP=""
ROUTER_IP=""

if [ -f "$CONFIG_DIR/robot_config.txt" ]; then
    source "$CONFIG_DIR/robot_config.txt"
    echo "Found robot configuration: $ROBOT_IP"
fi

if [ -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    source "$CONFIG_DIR/mitm_router_config.txt"
    echo "Found router configuration: $ROUTER_IP"
fi

# Configure ROS_DOMAIN_ID (default to 0)
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=0
    echo "Using ROS_DOMAIN_ID=0 (default)"
fi

# Configure DDS for cross-machine discovery (if robot IP is known)
if [ -n "$ROBOT_IP" ] && [ -n "$ROUTER_IP" ]; then
    echo ""
    echo "Step 1: Configuring DDS for cross-machine discovery..."
    
    # Create DDS config for peer discovery
    DDS_CONFIG_FILE="$CONFIG_DIR/dds_controller_config.xml"
    mkdir -p "$CONFIG_DIR"
    
    cat > "$DDS_CONFIG_FILE" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>$ROS_DOMAIN_ID</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="$ROBOT_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    
    export CYCLONEDDS_URI="file://$DDS_CONFIG_FILE"
    echo "✅ DDS configuration created: $DDS_CONFIG_FILE"
    echo "   Configured to discover robot at: $ROBOT_IP"
    
    # Restart ROS2 daemon to apply DDS config
    echo "Restarting ROS2 daemon..."
    ros2 daemon stop 2>/dev/null
    sleep 1
    ros2 daemon start
    sleep 2
    echo "✅ ROS2 daemon restarted"
else
    echo ""
    echo "⚠️  Robot IP not found in configuration"
    echo "   Will try default DDS discovery (multicast)"
    echo "   If discovery fails, ensure ROS_DOMAIN_ID matches on both machines"
fi

echo ""
echo "Step 2: Checking ROS2 topic discovery..."
echo "Waiting for DDS to discover remote nodes (this may take 10-15 seconds)..."
echo ""

# Wait and check for topics multiple times
MAX_RETRIES=6
RETRY_COUNT=0
TOPIC_FOUND=0

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    sleep 3
    RETRY_COUNT=$((RETRY_COUNT + 1))
    
    if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
        echo "✅ /cmd_vel topic discovered!"
        TOPIC_FOUND=1
        break
    else
        echo "   Attempt $RETRY_COUNT/$MAX_RETRIES: Waiting for /cmd_vel topic..."
        # Show discovered topics for debugging
        DISCOVERED_TOPICS=$(ros2 topic list 2>/dev/null)
        if [ -n "$DISCOVERED_TOPICS" ]; then
            echo "   Discovered topics: $(echo "$DISCOVERED_TOPICS" | tr '\n' ' ')"
        fi
    fi
done

if [ $TOPIC_FOUND -eq 0 ]; then
    echo ""
    echo "❌ /cmd_vel topic not found after $MAX_RETRIES attempts"
    echo ""
    echo "Troubleshooting steps:"
    echo "  1. Verify Gazebo is running on robot machine"
    echo "  2. Check ROS_DOMAIN_ID matches on both machines (current: $ROS_DOMAIN_ID)"
    if [ -n "$ROBOT_IP" ]; then
        echo "  3. Test network connectivity: ping $ROBOT_IP"
        echo "  4. Check if robot machine can be reached: ros2 node list"
    fi
    echo "  5. Verify MITM router is forwarding traffic correctly"
    echo "  6. Check firewall on DDS ports (7400-7500)"
    echo ""
    echo "Current ROS2 environment:"
    echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    if [ -n "$CYCLONEDDS_URI" ]; then
        echo "  CYCLONEDDS_URI: $CYCLONEDDS_URI"
    fi
    echo ""
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
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

