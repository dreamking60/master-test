# How to View Attack Logs

## Yes, you can see attack information in the logs!

The attack script `injection_attack.py` uses ROS2's logging system to record all attack activities. Here are several methods to view attack logs.

## Method 1: View ROS2 Log Files (Recommended)

### Find log files containing attack information

```bash
# Find log files containing "attack" or "injection_attacker"
find ~/.ros/log -name "*.log" -exec grep -l "injection_attacker\|ATTACK\|attack" {} \;
```

### View latest attack log

```bash
# View latest log file (sorted by time)
find ~/.ros/log -name "python3_*.log" -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2- | xargs cat
```

### Search for attack-related information

```bash
# Search all attack-related logs
grep -i "attack\|injection_attacker" ~/.ros/log/*.log

# Search for specific attack types
grep -i "turn_left\|override\|spin" ~/.ros/log/*.log
```

## Method 2: Use rqt_console (GUI)

```bash
# Install (if not installed)
sudo apt install ros-jazzy-rqt-console  # or ros-humble-rqt-console

# Launch log viewer
ros2 run rqt_console rqt_console
```

In rqt_console:
- View all node logs in real-time
- Filter by log level (DEBUG, INFO, WARN, ERROR)
- Search for "injection_attacker" node
- Save logs to file

## Method 3: Real-time Attack Log Monitoring

### View in real-time while running attack

```bash
# Run attack in one terminal
cd /home/stevenchen/master-test/attack_experiments/scripts
python3 injection_attack.py --attack-type turn_left --frequency 50

# Monitor logs in real-time in another terminal
tail -f ~/.ros/log/latest/*.log | grep -i "attack\|injection"
```

## Method 4: Save Attack Logs to File

### Modify experiment script to save attack logs

When running attacks, you can redirect output to a file:

```bash
# Save attack log to file
cd /home/stevenchen/master-test/attack_experiments/scripts
python3 injection_attack.py \
    --attack-type turn_left \
    --frequency 50 \
    --duration 15 \
    --angular-speed 0.5 \
    --node-name "injection_attacker" \
    > /tmp/attack_$(date +%Y%m%d_%H%M%S).log 2>&1

# View saved log
cat /tmp/attack_*.log
```

## Attack Log Examples

Typical attack log content:

```
[WARN] [1765235081.329284737] [injection_attacker]: ATTACK MODE: injection_attacker - trying to hijack control!
[WARN] [1765235081.329593325] [injection_attacker]: Starting turn-left attack: 50.0 Hz for 15.0s
[WARN] [1765235096.336509699] [injection_attacker]: Attack finished
[INFO] [1765235096.336122069] [injection_attacker]: Stopped
```

### Log Level Explanation

- **WARN**: Attack start/end, attack type and parameters
- **INFO**: Node startup, shutdown, and general information

## Method 5: Use Project's Built-in Log Viewer

```bash
# Use project-provided log viewer script
cd /home/stevenchen/master-test
./scripts/testing/view_logs.sh

# Then select:
# 4) Search for warnings (attack logs are usually WARN level)
# or
# 1) View latest ROS2 log file
```

## Viewing Attack Logs in SROS2 Security Tests

When using SROS2 security mechanisms, attacks may fail, but logs will still record:

```bash
# Run attack (should fail)
cd /home/stevenchen/master-test/attack_experiments/scripts
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
python3 injection_attack.py --attack-type turn_left --frequency 50

# View logs, may see connection failure error messages
grep -i "attack\|error\|security" ~/.ros/log/*.log | tail -20
```

## Quick Check Commands

```bash
# One-command to view recent attack logs
find ~/.ros/log -name "*.log" -type f -exec grep -l "injection_attacker" {} \; | \
    xargs ls -lt | head -1 | awk '{print $NF}' | xargs tail -20
```

## Log File Locations

- **ROS2 default log directory**: `~/.ros/log/`
- **Project log directory**: `/home/stevenchen/master-test/logs/` (if `ROS_LOG_DIR` is set)
- **Temporary logs**: `/tmp/attack_*.log` (if manually saved)

## Tips

1. **Log file name format**: `python3_<PID>_<timestamp>.log`
2. **Timestamp**: Timestamps in logs are Unix timestamps (seconds.nanoseconds)
3. **Node name**: Default attack node name is `injection_attacker`
4. **Real-time monitoring**: Use `tail -f` to view log updates in real-time
