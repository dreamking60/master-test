# MITM Router Setup Guide

## Overview

This guide explains how to set up a real Man-in-the-Middle (MITM) attack using a router-based approach.

## Network Topology

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Robot     │────────▶│ MITM Router  │────────▶│ Controller  │
│  (Target)   │         │  (Attacker)  │         │ (Legitimate)│
└─────────────┘         └──────────────┘         └─────────────┘
    192.168.1.10             192.168.1.1             192.168.1.20
    Gateway: 1.1              (Router)                Gateway: 1.1
```

## Setup Steps

### Step 1: Configure MITM Router (Attacker Machine)

**On attacker machine**:

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
sudo ./setup_mitm_router.sh
```

This will:
- Enable IP forwarding
- Configure iptables for NAT
- Set up routing rules
- Save configuration

**Result**: Attacker machine can now route traffic between robot and controller.

### Step 2: Configure Robot (Target Machine)

**On robot machine**:

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
sudo ./setup_robot_connection.sh
```

Enter:
- Network interface name
- MITM router IP address (as gateway)

**Result**: Robot routes all traffic through MITM router.

### Step 3: Configure Controller (Legitimate Machine)

**On controller machine**:

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
sudo ./setup_controller_connection.sh
```

Enter:
- Network interface name
- MITM router IP address (as gateway)

**Result**: Controller routes all traffic through MITM router.

### Step 4: Start Interception

**On MITM router (attacker machine)**:

```bash
sudo ./intercept_and_modify.sh
```

This will:
- Capture all DDS traffic
- Log intercepted packets
- Monitor ROS2 communication

## How It Works

### IP Forwarding

The MITM router enables IP forwarding:
```bash
echo 1 > /proc/sys/net/ipv4/ip_forward
```

This allows the machine to forward packets between interfaces.

### NAT (Network Address Translation)

iptables NAT rules forward traffic:
```bash
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
```

This makes the router transparent to both sides.

### Traffic Flow

1. **Robot sends message** → Router receives
2. **Router intercepts** → Can read/modify
3. **Router forwards** → To controller
4. **Controller receives** → (Possibly modified) message

## Attack Capabilities

### 1. Traffic Interception

All DDS traffic passes through router:
- Can capture all packets
- Can analyze message content (if unencrypted)
- Can log all communication

### 2. Message Modification

If traffic is unencrypted:
- Can modify `/cmd_vel` messages
- Can inject malicious commands
- Can drop legitimate commands

### 3. Traffic Analysis

Can analyze:
- Message frequency
- Node communication patterns
- Topic usage

## Limitations

### With Unencrypted ROS2 (Default)

- ✅ Can intercept messages
- ✅ Can read message content
- ✅ Can modify messages
- ✅ Can inject commands

### With SROS2 Enabled

- ✅ Can intercept packets (encrypted)
- ❌ Cannot read message content (encrypted)
- ❌ Cannot modify messages (encryption + signature)
- ⚠️  Can still drop/block traffic

## Verification

### Check Router Status

```bash
# Check IP forwarding
cat /proc/sys/net/ipv4/ip_forward  # Should be 1

# Check iptables rules
sudo iptables -t nat -L -n -v

# Check routing
ip route show
```

### Test Connectivity

**On robot machine**:
```bash
ping <controller_ip>  # Should work through router
```

**On controller machine**:
```bash
ping <robot_ip>  # Should work through router
```

### Test ROS2 Communication

**On robot machine**:
```bash
ros2 topic list  # Should see topics from controller
```

**On controller machine**:
```bash
ros2 topic list  # Should see topics from robot
```

## Security Implications

### Default ROS2 (Unencrypted)

- **Vulnerable** to MITM attacks
- Messages can be read and modified
- No protection against interception

### SROS2 (Encrypted)

- **Protected** against message reading
- **Protected** against message modification
- **Still vulnerable** to traffic blocking/dropping

## Cleanup

To restore normal network:

**On robot machine**:
```bash
# Restore original gateway
sudo ip route del default
sudo ip route add default via <original_gateway>
```

**On controller machine**:
```bash
# Restore original gateway
sudo ip route del default
sudo ip route add default via <original_gateway>
```

**On MITM router**:
```bash
# Disable IP forwarding
echo 0 > /proc/sys/net/ipv4/ip_forward

# Clear iptables rules
sudo iptables -t nat -F
```

## Troubleshooting

### No Traffic Through Router

- Check IP forwarding is enabled
- Check iptables rules
- Verify gateway configuration on both machines
- Check firewall rules

### Cannot Intercept Traffic

- Ensure running as root
- Check interface name is correct
- Verify traffic is actually flowing through router
- Check tcpdump permissions

### ROS2 Nodes Not Discovered

- Verify ROS_DOMAIN_ID matches
- Check network connectivity
- Ensure ROS2 daemon is running
- Check firewall on DDS ports (7400-7500)

