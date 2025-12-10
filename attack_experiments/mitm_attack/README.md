# Real Man-in-the-Middle (MITM) Attack Experiment

## Overview

This experiment implements a **real MITM attack** using a network router as the man-in-the-middle. The attacker machine acts as a router, and both the robot and legitimate controller connect through it, allowing the attacker to intercept, modify, and forward traffic.

**Attack Layer**: **Application Layer MITM** (with network layer routing support)

The attack uses network layer routing to position the attacker in the middle, but message modification happens at the ROS2 application layer using a ROS2 node. This is a practical hybrid approach that effectively demonstrates MITM attacks.

## Network Topology

```
[Robot Machine] <---> [MITM Router/Attacker] <---> [Controller Machine]
                            |
                            └──> Intercepts, modifies, forwards traffic
```

## Attack Scenario

1. **Setup**: Attacker machine configured as router
2. **Connection**: Robot and controller connect through attacker
3. **Interception**: Attacker intercepts all DDS traffic
4. **Modification**: Attacker can modify `/cmd_vel` messages
5. **Forwarding**: Attacker forwards (possibly modified) traffic

## Prerequisites

1. **Three machines or VMs**:
   - Robot machine (target)
   - Controller machine (legitimate operator)
   - Attacker machine (MITM router)

2. **Network configuration**:
   - All machines on same network segment
   - Attacker machine has IP forwarding enabled
   - Attacker can route traffic between robot and controller

3. **Tools**:
   - `iptables` for packet forwarding
   - `tcpdump` or `wireshark` for packet capture
   - Network configuration tools

## Attack Layer Classification

**Current Implementation**: **Application Layer MITM** with network layer support

- **Network Layer**: Router setup enables traffic routing
- **Application Layer**: Python ROS2 node modifies messages
- **Hybrid Approach**: Combines both layers for practical attack

See `docs/ATTACK_LAYERS.md` for detailed layer analysis.

## Attack Methods

### Method 1: Router-Based MITM (Current Implementation)

Attacker machine acts as actual router:
- Robot's default gateway → Attacker
- Controller's default gateway → Attacker
- All traffic flows through attacker
- **Application layer**: ROS2 node modifies messages

### Method 2: ARP Spoofing MITM (Network Layer)

Attacker uses ARP spoofing to intercept:
- Attacker spoofs router's MAC address
- Traffic redirected to attacker
- Attacker forwards traffic
- **True network layer** attack

### Method 3: Bridge Mode MITM (Network Layer)

Attacker creates network bridge:
- Transparent bridge between robot and controller
- All traffic passes through bridge
- Attacker can intercept/modify at packet level
- **True network layer** attack

## Files

- `scripts/setup_mitm_router.sh` - Configure attacker as router
- `scripts/setup_robot_connection.sh` - Configure robot to use MITM router
- `scripts/setup_controller_connection.sh` - Configure controller to use MITM router
- `scripts/intercept_and_modify.sh` - Intercept and log DDS traffic
- `scripts/modify_ros2_messages.py` - Python script to modify ROS2 messages
- `scripts/launch_mitm_attack.sh` - Launch complete MITM attack with message modification
- `scripts/test_mitm_attack.sh` - Complete MITM attack test
- `docs/MITM_ROUTER_SETUP.md` - Detailed router setup guide
- `docs/ATTACK_FLOW.md` - Attack flow and methodology

## Quick Start

### Step 1: Setup MITM Router (Attacker Machine)

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
./setup_mitm_router.sh
```

### Step 2: Configure Robot (Target Machine)

```bash
./setup_robot_connection.sh
# Enter attacker's IP as gateway
```

### Step 3: Configure Controller (Legitimate Machine)

```bash
./setup_controller_connection.sh
# Enter attacker's IP as gateway
```

### Step 4: Launch MITM Attack

```bash
# On attacker machine
./intercept_and_modify.sh
```

## Expected Results

With **unencrypted ROS2** (default):
- ✅ MITM attack should **SUCCEED**
- ✅ Attacker can intercept traffic
- ✅ Attacker can modify messages
- ✅ Robot receives modified commands

With **SROS2 enabled**:
- ❌ Traffic is encrypted
- ❌ Cannot read/modify encrypted messages
- ⚠️  But attacker can still drop/block traffic

## Security Implications

This demonstrates:
- **Default ROS2 is vulnerable** to MITM attacks
- **SROS2 encryption** protects message content
- **Network security** is important even with SROS2
- **Physical network access** enables MITM attacks

