# Network Attack Scripts Guide

## Available Attack Scripts

### 1. launch_attack_interactive.sh (Recommended)

**Full interactive attack launcher with security mode detection**

Features:
- Detects if SROS2 security is enabled
- Interactive menu for attack selection
- Parameter configuration
- Connection verification
- Works with both secure and non-secure systems

Usage:
```bash
cd attack_experiments/network_attack/scripts
./launch_attack_interactive.sh
```

### 2. quick_attack.sh

**Quick one-command attack**

Usage:
```bash
# With parameters
./quick_attack.sh [attack_type] [frequency] [duration]

# Examples
./quick_attack.sh turn_left 50 15
./quick_attack.sh override 50 20
./quick_attack.sh spin 30 10

# Use defaults (turn_left, 50 Hz, 15s)
./quick_attack.sh
```

### 3. launch_attack.sh

**Original interactive script (updated)**

Similar to `launch_attack_interactive.sh` but simpler interface.

## Attack Types

| Type | Description | Default Frequency | Default Duration |
|------|-------------|-------------------|------------------|
| `turn_left` | Forces robot to turn left | 50 Hz | 15s |
| `override` | Forces robot forward | 50 Hz | 15s |
| `spin` | Makes robot spin | 30 Hz | 10s |
| `stealth` | Lower frequency attack | 25 Hz | 30s |
| `interference` | Random commands | 40 Hz | 30s |

## Security Mode Support

All scripts automatically detect if SROS2 security is enabled:

- **Security Enabled**: Scripts will show warning and attempt attack (will fail if no valid credentials)
- **Security Disabled**: Scripts work normally

## Examples

### Example 1: Quick Turn Left Attack

```bash
./quick_attack.sh turn_left
```

### Example 2: Custom Override Attack

```bash
./quick_attack.sh override 100 30
# 100 Hz frequency, 30 seconds duration
```

### Example 3: Interactive with Custom Parameters

```bash
./launch_attack_interactive.sh
# Select option 6 (Custom)
# Enter parameters when prompted
```

## Script Comparison

| Feature | launch_attack_interactive.sh | quick_attack.sh | launch_attack.sh |
|---------|----------------------------|-----------------|------------------|
| Interactive menu | ✅ | ❌ | ✅ |
| Command-line args | ❌ | ✅ | ❌ |
| Security detection | ✅ | ✅ | ✅ |
| Connection check | ✅ | ❌ | ✅ |
| Parameter config | ✅ | Limited | ✅ |

## Recommended Usage

- **First time / Testing**: Use `launch_attack_interactive.sh`
- **Quick tests**: Use `quick_attack.sh`
- **Automation**: Use `quick_attack.sh` with parameters

