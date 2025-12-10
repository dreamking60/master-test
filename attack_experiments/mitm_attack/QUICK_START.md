# MITM Attack - Quick Start Guide

## Overview

Real Man-in-the-Middle attack using router-based interception.

## Network Setup

```
Robot (192.168.1.10) <---> MITM Router (192.168.1.1) <---> Controller (192.168.1.20)
```

## Quick Steps

### Step 1: Setup MITM Router (Attacker Machine)

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
sudo ./setup_mitm_router.sh
```

**Result**: Attacker machine configured as router with IP forwarding.

### Step 2: Configure Robot (Target Machine)

```bash
sudo ./setup_robot_connection.sh
# Enter network interface and router IP
```

**Result**: Robot routes all traffic through MITM router.

### Step 3: Configure Controller (Legitimate Machine)

```bash
sudo ./setup_controller_connection.sh
# Enter network interface and router IP
```

**Result**: Controller routes all traffic through MITM router.

### Step 4: Start MITM Attack

**On MITM router machine**:

**Option A: Intercept and modify messages** (for unencrypted ROS2):
```bash
./launch_mitm_attack.sh
# Select modification type (reverse, stop, spin, amplify)
```

**Option B: Just intercept and log**:
```bash
sudo ./intercept_and_modify.sh
```

**Result**: 
- Option A: Messages are intercepted, modified, and forwarded
- Option B: All DDS traffic is intercepted and logged

### Step 5: Test Attack

**On controller machine**:
```bash
# Start normal controller
python3 normal_controller.py --pattern continuous
```

**On MITM router**:
- Observe intercepted traffic
- Analyze captured packets
- Test message modification (if unencrypted)

## Expected Results

### Default ROS2 (Unencrypted)

- ✅ Can intercept all messages
- ✅ Can read message content
- ✅ Can modify `/cmd_vel` commands
- ✅ Attack succeeds

### SROS2 Enabled

- ✅ Can intercept packets (encrypted)
- ❌ Cannot read content
- ❌ Cannot modify (signature fails)
- ⚠️  Can still block traffic

## Verification

Check if traffic flows through router:

```bash
# On router machine
sudo tcpdump -i <interface> 'udp portrange 7400-7500'
```

Should see DDS packets flowing through.

## Cleanup

To restore normal network:

**On robot/controller**:
```bash
# Restore original gateway
sudo ip route del default
sudo ip route add default via <original_gateway_ip>
```

**On router**:
```bash
# Disable forwarding
echo 0 > /proc/sys/net/ipv4/ip_forward
sudo iptables -t nat -F
```

