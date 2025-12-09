# Attack Connection Monitoring

## Overview

These scripts help you **monitor and log** attack connections from other devices in real-time.

**Important**: These scripts are **monitoring tools only** - they do NOT intercept, block, or prevent attacks. They only observe and record. Actual security and defense should be handled by SROS2 (Secure ROS2).

## Scripts

### 1. monitor_attack_connections.sh

**Real-time monitoring script** - Monitors `/cmd_vel` topic for suspicious activity.

**Usage**:
```bash
cd /home/stevenchen/master-test
./scripts/monitoring/monitor_attack_connections.sh
```

**What it monitors**:
- New publishers on `/cmd_vel` topic
- Unexpected node names (attack indicators)
- High message frequency (potential attack)
- Suspicious node names (containing "attack", "injection", etc.)

**Output**:
- Logs all events to `logs/monitoring/attack_connections_YYYYMMDD_HHMMSS.log`
- Shows real-time alerts in terminal
- Timestamps all events

### 2. view_attack_connections.sh

**View recorded logs** - Shows attack connection history.

**Usage**:
```bash
cd /home/stevenchen/master-test
./scripts/monitoring/view_attack_connections.sh
```

**What it shows**:
- Latest monitoring log
- All alerts and warnings
- Summary statistics

## How It Works

**This is a monitoring tool only** - it observes and logs, but does NOT:
- ❌ Block or intercept messages
- ❌ Prevent attacks
- ❌ Stop malicious nodes
- ❌ Modify ROS2 communication

**For actual security**, use SROS2 (Secure ROS2) which provides:
- ✅ Authentication (certificates)
- ✅ Encryption
- ✅ Access control (policies)
- ✅ Message integrity

### Detection Methods

1. **Publisher Monitoring** (Read-only):
   - Observes who is publishing to `/cmd_vel`
   - Alerts when unexpected nodes appear
   - Known legitimate nodes: `robot_state_publisher`, `teleop`, `normal_operator`, `controller`

2. **Frequency Detection** (Read-only):
   - Normal operation: 10-20 Hz
   - Attack: 50+ Hz
   - Alerts when frequency exceeds 30 Hz

3. **Node Name Analysis** (Read-only):
   - Checks for suspicious keywords in node names
   - Keywords: "attack", "injection", "hack", "malicious"

## Example Output

```
[2025-01-09 14:30:15] === Monitoring Started ===
[2025-01-09 14:30:15] Current ROS_DOMAIN_ID: 0
[2025-01-09 14:30:15] Current machine IP: 192.168.1.100

[2025-01-09 14:30:20] ⚠️  Publisher count changed: 2
[2025-01-09 14:30:20] Publisher nodes:
[2025-01-09 14:30:20]   - /robot_state_publisher
[2025-01-09 14:30:20]   - /injection_attacker
[2025-01-09 14:30:20] 🚨 ALERT: Unexpected publisher detected: /injection_attacker
[2025-01-09 14:30:20]    This might be an attack!

[2025-01-09 14:30:25] ⚠️  High message frequency detected: 50.0 Hz (potential attack)
```

## Quick Start

### Start Monitoring (Terminal 1)

```bash
cd /home/stevenchen/master-test
./scripts/monitoring/monitor_attack_connections.sh
```

### View Logs (Terminal 2)

```bash
cd /home/stevenchen/master-test
./scripts/monitoring/view_attack_connections.sh
```

## Integration with ROS2 Logs

The monitoring script complements ROS2's built-in logging:
- ROS2 logs: Node-level messages and errors
- Monitoring logs: Connection-level events and alerts

Both can be viewed together for complete attack analysis.

## Notes

- **Monitoring only** - Does not interfere with ROS2 communication
- Monitoring runs continuously until stopped (Ctrl+C)
- Logs are saved automatically
- Multiple monitoring sessions can run simultaneously
- Logs are timestamped for correlation with other events

## Security vs Monitoring

| Feature | Monitoring Script | SROS2 (Secure ROS2) |
|---------|------------------|---------------------|
| Detect attacks | ✅ Yes | ✅ Yes |
| Log events | ✅ Yes | ✅ Yes |
| Block attacks | ❌ No | ✅ Yes |
| Encrypt messages | ❌ No | ✅ Yes |
| Authenticate nodes | ❌ No | ✅ Yes |
| Access control | ❌ No | ✅ Yes |

**Use monitoring for**: Observation, logging, analysis, research
**Use SROS2 for**: Actual security, production systems, defense

