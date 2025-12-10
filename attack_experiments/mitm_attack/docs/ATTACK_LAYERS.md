# MITM Attack Layers Analysis

## Current Implementation Analysis

The current MITM attack implementation is a **hybrid approach** combining network layer and application layer:

### Current Implementation

```
┌─────────────────────────────────────────────────┐
│  Network Layer (Router Setup)                  │
│  - IP forwarding                                │
│  - Packet routing                               │
│  - Traffic interception                         │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│  Application Layer (Python Script)              │
│  - ROS2 node subscribes to /cmd_vel             │
│  - Modifies message content                     │
│  - Republishes modified message                  │
└─────────────────────────────────────────────────┘
```

## Attack Layer Breakdown

### 1. Network Layer Component

**What it does**:
- Router setup (`setup_mitm_router.sh`)
- IP forwarding enabled
- Traffic routing through attacker
- Packet capture (`tcpdump`)

**Layer**: **Network Layer** (Layer 3)
- Operates at IP packet level
- Routes packets between networks
- Can intercept all network traffic

**Limitation**: 
- Can capture packets
- But cannot easily modify ROS2 message content at packet level
- Would need to decode DDS protocol

### 2. Application Layer Component

**What it does**:
- Python ROS2 node (`modify_ros2_messages.py`)
- Subscribes to `/cmd_vel` topic
- Modifies message content
- Republishes modified message

**Layer**: **Application Layer** (Layer 7)
- Operates at ROS2 message level
- Uses ROS2 API (rclpy)
- Can modify message content easily

**Requirement**: 
- Must be able to run ROS2 nodes
- Must be on same ROS2 network
- Requires ROS2 environment

## True Network Layer MITM

A **true network layer MITM** would:

1. **Intercept at packet level**:
   - Capture raw IP/UDP packets
   - Parse DDS protocol
   - Modify packet payload
   - Resend modified packets

2. **No ROS2 dependency**:
   - Doesn't need ROS2 installed
   - Works at network protocol level
   - Pure packet manipulation

3. **Transparent to applications**:
   - Applications don't know about MITM
   - No need to run ROS2 nodes
   - Works for any protocol

## Comparison

| Feature | Current (Hybrid) | True Network Layer |
|---------|-----------------|-------------------|
| **Layer** | Network + Application | Network only |
| **Requires ROS2** | ✅ Yes | ❌ No |
| **Packet level** | ✅ Can capture | ✅ Can capture & modify |
| **Message level** | ✅ Easy to modify | ⚠️  Need DDS parsing |
| **Transparency** | ⚠️  Visible as ROS2 node | ✅ Transparent |
| **Complexity** | ⭐⭐ Medium | ⭐⭐⭐ Hard |

## Current Implementation Classification

**Primary Layer**: **Application Layer**
- The message modification happens at ROS2 application level
- Requires ROS2 node to subscribe/publish
- Uses ROS2 API

**Supporting Layer**: **Network Layer**
- Router setup enables traffic flow through attacker
- But actual modification is at application level

## True Network Layer MITM (Advanced)

For a **true network layer MITM**, you would need:

1. **DDS Protocol Parser**:
   - Parse DDS RTPS (Real-Time Publish-Subscribe) protocol
   - Extract message content from packets
   - Reconstruct ROS2 messages

2. **Packet Modification**:
   - Modify packet payload
   - Recalculate checksums
   - Resend modified packets

3. **Tools**:
   - `scapy` (Python packet manipulation)
   - Custom DDS parser
   - Network bridge/tap interface

## Recommendation

**Current implementation is Application Layer MITM** with network layer support.

For **true network layer MITM**, you would need:
- DDS protocol knowledge
- Packet-level manipulation tools
- More complex implementation

The current approach is **practical and effective** for demonstrating MITM attacks, even though it operates primarily at the application layer.

