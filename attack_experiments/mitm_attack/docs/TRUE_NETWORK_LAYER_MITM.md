# True Network Layer MITM Attack

## Overview

A **true network layer MITM** operates entirely at the network/packet level, without requiring ROS2 or application-level access.

## Difference from Current Implementation

### Current Implementation (Application Layer)

```
[Controller] → [Router] → [Robot]
                    ↓
              [ROS2 Node modifies messages]
```

- Requires ROS2 environment
- Uses ROS2 API (rclpy)
- Visible as ROS2 node
- Easy to implement

### True Network Layer MITM

```
[Controller] → [Router] → [Robot]
                    ↓
              [Packet-level modification]
```

- No ROS2 required
- Operates at IP/UDP packet level
- Transparent to applications
- More complex to implement

## True Network Layer MITM Requirements

### 1. DDS Protocol Knowledge

DDS uses RTPS (Real-Time Publish-Subscribe) protocol:
- RTPS messages in UDP packets
- Need to parse RTPS headers
- Extract message payload
- Modify payload
- Reconstruct packets

### 2. Packet Manipulation Tools

- **scapy** (Python): Packet crafting and manipulation
- **libpcap**: Packet capture library
- **Custom DDS parser**: Parse RTPS protocol

### 3. Network Interface

- Bridge mode or TAP interface
- Transparent packet forwarding
- Packet modification capability

## Implementation Approach

### Option 1: Using Scapy (Python)

```python
from scapy.all import *
import struct

def modify_dds_packet(packet):
    """Modify DDS packet at network level"""
    if packet.haslayer(UDP):
        # Parse DDS/RTPS protocol
        # Modify payload
        # Reconstruct packet
        pass
```

### Option 2: Using iptables + netfilter

```bash
# Redirect packets to userspace
iptables -t mangle -A PREROUTING -p udp --dport 7400:7500 -j NFQUEUE --queue-num 0

# Process in userspace application
# Modify packets
# Resend
```

### Option 3: Using eBPF/XDP

```c
// eBPF program to modify packets
// Attach to network interface
// Modify packets in kernel space
```

## Challenges

### 1. DDS Protocol Complexity

- RTPS is complex protocol
- Multiple message types
- Submessage headers
- Data serialization (CDR)

### 2. Message Reconstruction

- Need to parse CDR (Common Data Representation)
- Understand ROS2 message format
- Reconstruct after modification

### 3. Packet Integrity

- Recalculate checksums
- Maintain packet ordering
- Handle fragmentation

## Why Current Implementation is Practical

The current **application layer** approach is more practical because:

1. **Easier to implement**: Uses ROS2 API directly
2. **No protocol parsing**: ROS2 handles DDS
3. **Message-level access**: Direct access to message content
4. **Effective demonstration**: Shows MITM attack concept

## When True Network Layer is Needed

True network layer MITM is needed when:

1. **No ROS2 access**: Cannot run ROS2 nodes
2. **Protocol-agnostic**: Need to attack any protocol
3. **Stealth requirement**: Must be invisible to applications
4. **Research purpose**: Studying network-level attacks

## Current Implementation Classification

**Layer**: **Application Layer** (with network layer support)

**Why**: 
- Message modification happens via ROS2 API
- Requires ROS2 node to run
- Operates at ROS2 message level

**Network layer component**:
- Router setup enables traffic flow
- But actual attack is at application level

## Conclusion

The current MITM implementation is **Application Layer MITM** that uses network layer routing to position the attacker in the middle. This is a practical and effective approach for demonstrating MITM attacks on ROS2 systems.

For a **true network layer MITM**, you would need DDS protocol parsing and packet-level manipulation, which is significantly more complex.

