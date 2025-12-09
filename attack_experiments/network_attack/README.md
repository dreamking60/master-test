# Network Layer Attack Experiment

This experiment demonstrates network-layer injection attacks on ROS2 robots. The attacker is on a different machine and attacks the target robot over the network using ROS2's DDS discovery mechanism.

## Overview

- **Target Machine**: Runs Gazebo simulation with the robot
- **Attacker Machine**: Discovers and attacks the robot over the network
- **Attack Method**: Exploits DDS automatic discovery (multicast or peer-to-peer)

## Prerequisites

1. **Two machines on the same network** (or configured for DDS routing)
2. **ROS2 installed on both machines** (same version recommended)
3. **Network connectivity** between machines

## Quick Start

### Step 1: Setup Target Machine

On the **target machine** (the one running Gazebo):

```bash
cd attack_experiments/network_attack/scripts
chmod +x setup_target_machine.sh
./setup_target_machine.sh
```

This will:
- Configure ROS_DOMAIN_ID
- Get the machine's IP address
- Start Gazebo simulation
- Save configuration for sharing

**Note the IP address and ROS_DOMAIN_ID** - you'll need these for the attacker machine.

### Step 2: Setup Attacker Machine

On the **attacker machine** (different machine):

```bash
cd attack_experiments/network_attack/scripts
chmod +x setup_attacker_machine.sh
./setup_attacker_machine.sh
```

When prompted:
- Enter the target machine's IP address
- Enter the target machine's ROS_DOMAIN_ID
- The script will test connectivity and configure DDS if needed

### Step 3: Launch Attack

On the **attacker machine**:

**Option A: Interactive Attack (Recommended)**
```bash
cd attack_experiments/network_attack/scripts
./launch_attack_interactive.sh
```
Select attack type from menu and configure parameters.

**Option B: Quick Attack**
```bash
cd attack_experiments/network_attack/scripts
./quick_attack.sh turn_left 50 15
# Or use defaults: ./quick_attack.sh
```

**Option C: Original Script**
```bash
cd attack_experiments/network_attack/scripts
./launch_attack.sh
```

Watch the robot in Gazebo on the target machine!

## Attack Types

1. **Turn Left** - Forces robot to turn left (default)
2. **Override** - Forces robot to move forward
3. **Spin** - Makes robot spin rapidly
4. **Stealth** - Lower frequency attack (harder to detect)

## Manual Attack

You can also run attacks manually:

```bash
# On attacker machine
cd attack_experiments/network_attack/scripts
python3 ../../scripts/injection_attack.py \
    --attack-type turn_left \
    --frequency 50 \
    --duration 15 \
    --angular-speed 0.5
```

## Network Scenarios

### Same Subnet (Easiest)

If both machines are on the same subnet:
- DDS multicast will work automatically
- No special configuration needed
- Just set same ROS_DOMAIN_ID

### Different Subnets

If machines are on different subnets:
- DDS routing configuration is created automatically
- Uses peer-to-peer discovery instead of multicast
- Configuration saved in `dds_attack_config.xml`

## Troubleshooting

### Target Not Discovered

1. **Check network connectivity**:
   ```bash
   ping <target_ip>
   ```

2. **Verify ROS_DOMAIN_ID matches**:
   ```bash
   echo $ROS_DOMAIN_ID  # Should be same on both machines
   ```

3. **Check ROS2 daemon**:
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

4. **Verify topics**:
   ```bash
   ros2 topic list  # Should see /cmd_vel
   ```

### Attack Not Working

1. **Check if target is running Gazebo**
2. **Verify /cmd_vel topic exists**: `ros2 topic list | grep cmd_vel`
3. **Check message flow**: `ros2 topic echo /cmd_vel`
4. **Increase attack frequency** (try 50 Hz or higher)

### DDS Configuration Issues

If using DDS routing (different subnets):
- Check `dds_attack_config.xml` exists
- Verify `CYCLONEDDS_URI` is set: `echo $CYCLONEDDS_URI`
- Restart ROS2 daemon after setting environment variable

## Files

- `setup_target_machine.sh` - Setup script for target machine
- `setup_attacker_machine.sh` - Setup script for attacker machine
- `launch_attack.sh` - Interactive attack launcher
- `target_ip.txt` - Saved target IP (auto-generated)
- `target_domain_id.txt` - Saved domain ID (auto-generated)
- `dds_attack_config.xml` - DDS routing config (if needed)

## Security Implications

This experiment demonstrates:
- **DDS automatic discovery** allows any machine on the network to find ROS2 nodes
- **No authentication** by default - any node can publish to topics
- **Network isolation needed** for production systems
- **ROS_DOMAIN_ID** provides basic isolation but is not secure

## Defense Measures

1. **Use non-default ROS_DOMAIN_ID**: `export ROS_DOMAIN_ID=42`
2. **Network isolation**: Use firewall to restrict DDS ports (7400-7500/udp)
3. **SROS2**: Use Secure ROS2 for encryption and authentication
4. **VPN**: Use VPN for remote access instead of direct network connection
5. **Monitoring**: Monitor for unexpected publishers on critical topics

## Notes

- This is for **educational and security research purposes only**
- Always obtain proper authorization before testing
- Use isolated network environments
- The attack works because ROS2/DDS has no authentication by default

