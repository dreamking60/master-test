# MITM Attack Flow

## Attack Scenario

A real Man-in-the-Middle attack where the attacker machine acts as a router, intercepting all traffic between the robot and legitimate controller.

## Attack Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    MITM Attack Flow                         │
└─────────────────────────────────────────────────────────────┘

1. Setup Phase:
   ┌──────────┐         ┌──────────┐         ┌──────────┐
   │  Robot   │         │  MITM    │         │Controller│
   │          │         │  Router  │         │          │
   └──────────┘         └──────────┘         └──────────┘
        │                    │                    │
        │  Setup Router      │                    │
        │───────────────────▶│                    │
        │                    │                    │
        │  Configure Gateway │                    │
        │───────────────────▶│                    │
        │                    │                    │
        │                    │  Configure Gateway │
        │                    │◀───────────────────│
        │                    │                    │
        │                    │                    │
        │                    │                    │
2. Normal Communication (Intercepted):
        │                    │                    │
        │  /cmd_vel msg      │                    │
        │───────────────────▶│                    │
        │                    │  INTERCEPT         │
        │                    │  (read/modify)     │
        │                    │                    │
        │                    │  /cmd_vel msg      │
        │                    │  (possibly modified)│
        │                    │───────────────────▶│
        │                    │                    │
        │                    │                    │
3. Attack Phase:
        │                    │                    │
        │  Legitimate cmd    │                    │
        │───────────────────▶│                    │
        │                    │  MODIFY            │
        │                    │  (change to attack)│
        │                    │                    │
        │                    │  Malicious cmd     │
        │                    │───────────────────▶│
        │                    │                    │
        │                    │                    │
```

## Detailed Attack Steps

### Phase 1: Setup

1. **Attacker configures router**:
   - Enables IP forwarding
   - Configures iptables
   - Sets up NAT

2. **Robot connects to router**:
   - Changes default gateway to router
   - All traffic routes through router

3. **Controller connects to router**:
   - Changes default gateway to router
   - All traffic routes through router

### Phase 2: Interception

1. **Traffic flows through router**:
   - Robot → Router → Controller
   - Controller → Router → Robot

2. **Router intercepts packets**:
   - Captures all DDS traffic
   - Logs packet contents
   - Analyzes message patterns

### Phase 3: Modification (If Unencrypted)

1. **Router modifies messages**:
   - Intercepts `/cmd_vel` messages
   - Changes command values
   - Forwards modified messages

2. **Robot receives modified commands**:
   - Executes attacker's commands
   - Legitimate commands are blocked/modified

## Attack Capabilities

### 1. Passive Interception

- **Read all traffic**: Can see all ROS2 messages
- **Log communication**: Record all interactions
- **Analyze patterns**: Understand system behavior

### 2. Active Modification

- **Modify commands**: Change `/cmd_vel` values
- **Inject commands**: Add malicious commands
- **Block commands**: Drop legitimate messages

### 3. Selective Attack

- **Target specific topics**: Only modify `/cmd_vel`
- **Preserve other traffic**: Don't disrupt other communication
- **Stealth operation**: Hard to detect

## Defense Mechanisms

### Default ROS2 (No Defense)

- ❌ No encryption
- ❌ No authentication
- ❌ No message integrity
- ✅ **Vulnerable to MITM**

### SROS2 (With Defense)

- ✅ Encryption (TLS/DTLS)
- ✅ Authentication (certificates)
- ✅ Message integrity (signatures)
- ✅ **Protected against MITM**

## Expected Results

### With Unencrypted ROS2

**Attack succeeds**:
- ✅ Can intercept messages
- ✅ Can read message content
- ✅ Can modify commands
- ✅ Robot executes modified commands

### With SROS2 Enabled

**Attack partially succeeds**:
- ✅ Can intercept packets (encrypted)
- ❌ Cannot read content (encrypted)
- ❌ Cannot modify (signature verification fails)
- ⚠️  Can still drop/block traffic

## Testing Procedure

1. **Setup MITM router**
2. **Configure robot and controller**
3. **Start normal operation**
4. **Begin interception**
5. **Attempt modification** (if unencrypted)
6. **Observe results**

## Security Analysis

### What MITM Proves

- **Default ROS2**: Completely vulnerable
- **SROS2**: Protects content but not availability
- **Network security**: Important even with encryption

### Real-World Implications

- Physical network access enables MITM
- Encryption protects content, not routing
- Defense in depth is necessary

