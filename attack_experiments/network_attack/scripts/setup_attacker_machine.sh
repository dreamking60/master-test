#!/bin/bash
# Setup script for ATTACKER machine (the one that will attack)
# Run this on a different machine on the same network

echo "=========================================="
echo "Attacker Machine Setup - Network Attack Experiment"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Get target machine information
echo "Enter target machine information:"
read -p "Target machine IP address: " target_ip
read -p "Target machine ROS_DOMAIN_ID (default 0): " domain_id
domain_id=${domain_id:-0}

export ROS_DOMAIN_ID=$domain_id
echo ""
echo "ROS_DOMAIN_ID set to: $domain_id"
echo ""

# Test network connection
echo "Testing network connection to $target_ip..."
if ping -c 2 "$target_ip" &> /dev/null; then
    echo "Network connection OK"
else
    echo "Warning: Cannot ping target machine"
    echo "This might be normal if ping is disabled, but check network connectivity"
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
        exit 1
    fi
fi
echo ""

# Check if same subnet (for DDS multicast)
current_network=$(ip route | grep default | awk '{print $5}' | head -1)
target_network=$(echo "$target_ip" | cut -d'.' -f1-3)
current_network_prefix=$(ip addr show $current_network 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'.' -f1-3)

if [ "$current_network_prefix" = "$target_network" ]; then
    echo "Same subnet detected - DDS multicast should work"
    use_dds_config=false
else
    echo "Different subnet - DDS routing configuration needed"
    use_dds_config=true
fi
echo ""

# Create DDS config if needed
if [ "$use_dds_config" = "true" ]; then
    echo "Creating DDS routing configuration..."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    NETWORK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
    
    cat > "$NETWORK_DIR/dds_attack_config.xml" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>$domain_id</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="$target_ip"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    export CYCLONEDDS_URI=file://$NETWORK_DIR/dds_attack_config.xml
    echo "DDS config created: $NETWORK_DIR/dds_attack_config.xml"
    echo "Environment variable set: CYCLONEDDS_URI=$CYCLONEDDS_URI"
    echo ""
fi

# Restart ROS2 daemon
echo "Restarting ROS2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 2
echo ""

# Test discovery
echo "Testing node discovery..."
echo "Waiting 5 seconds for DDS to discover nodes..."
sleep 5

nodes=$(ros2 node list 2>/dev/null)
topics=$(ros2 topic list 2>/dev/null)

if [ -n "$nodes" ]; then
    echo "Nodes discovered:"
    echo "$nodes" | head -10
else
    echo "No nodes discovered yet"
    echo "This might be normal if target machine hasn't started Gazebo"
fi
echo ""

if echo "$topics" | grep -q cmd_vel; then
    echo "/cmd_vel topic discovered - ready to attack!"
else
    echo "/cmd_vel topic not found"
    echo "Make sure target machine has started Gazebo"
    read -p "Continue anyway? [y/N]: " continue_anyway
    if [ "$continue_anyway" != "y" ]; then
        exit 1
    fi
fi
echo ""

# Save configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NETWORK_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
echo "$target_ip" > "$NETWORK_DIR/target_ip.txt"
echo "$domain_id" > "$NETWORK_DIR/target_domain_id.txt"

echo "=========================================="
echo "Setup complete! Ready to attack."
echo "=========================================="
echo ""
echo "To launch attack, run:"
echo "  cd $SCRIPT_DIR"
echo "  ./launch_attack.sh"
echo ""
echo "Or manually:"
echo "  python3 ../scripts/injection_attack.py --attack-type turn_left --frequency 50 --duration 15"
echo ""

