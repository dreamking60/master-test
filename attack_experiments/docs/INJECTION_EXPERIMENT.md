# Application Layer Injection Attack Experiment

This experiment demonstrates a realistic attack scenario where a normal controller is operating the robot, and an attacker attempts to inject malicious commands to override the legitimate control.

## Experiment Scenario

1. **Normal Controller**: A legitimate operator controls the robot using a regular pattern (square, circle, etc.) at a normal frequency (e.g., 10 Hz)
2. **Attack Injection**: An attacker launches an injection attack at a higher frequency (e.g., 50 Hz) to try to override the normal control
3. **Observation**: Monitor whether the attack successfully hijacks the robot control

## Files

- `normal_controller.py` - Simulates legitimate user control
- `injection_attack.py` - Attack script attempting to override control
- `run_injection_experiment.sh` - Interactive experiment launcher
- `quick_test.sh` - Quick non-interactive test script

## Quick Start

### Prerequisites

1. Start Gazebo simulation:
```bash
cd /path/to/master-test
./scripts/setup/run.sh
```

2. Wait for Gazebo to fully load (you should see the robot in the world)

### Run Quick Test

```bash
cd attack_experiments
./quick_test.sh
```

This will:
- Start normal controller (square pattern, 10 Hz)
- Wait 5 seconds
- Launch injection attack (override, 50 Hz, 15 seconds)
- Stop normal controller after attack completes

### Run Interactive Experiment

```bash
cd attack_experiments
./run_injection_experiment.sh
```

This will prompt you for:
- Normal controller pattern (square/circle/forward_backward/continuous)
- Normal controller frequency
- Attack type (override/spin/interference/stealth)
- Attack frequency
- Attack timing

## Normal Controller Patterns

### Square Pattern
- Moves forward for 2 seconds
- Turns right for 2 seconds
- Repeats to form a square

### Circle Pattern
- Continuous forward with slight turn for circular motion

### Forward-Backward Pattern
- Moves forward for 2 seconds
- Moves backward for 2 seconds
- Repeats

### Continuous Pattern
- Continuous forward movement

## Attack Types

### Override Attack
- High-frequency publishing (default 50 Hz)
- Tries to dominate the `/cmd_vel` topic
- Sends forward commands to override normal control

### Spin Attack
- Rapid rotation commands
- Attempts to make robot spin uncontrollably

### Interference Attack
- Random commands to disrupt normal control
- Creates chaotic behavior

### Stealth Attack
- Moderate frequency (default 25 Hz)
- Less detectable but still attempts override

## Understanding the Results

### Successful Attack
- Robot behavior changes from normal pattern to attack pattern
- Attack commands override normal commands
- Robot follows attacker's commands instead of operator's

### Failed Attack
- Robot continues following normal pattern
- Attack commands are ignored or overridden by normal controller
- May indicate normal controller has higher priority or frequency

### Partial Success
- Robot behavior becomes erratic
- Mix of normal and attack commands
- Indicates competition between controllers

## Key Factors

1. **Frequency**: Higher frequency gives better chance to override
   - Normal: 10 Hz
   - Attack: 50 Hz (5x higher)

2. **Timing**: When attack starts relative to normal controller

3. **Message Queue**: ROS2 topic queue size (default 10)
   - Higher frequency can fill queue faster

4. **Network Latency**: In real scenarios, network conditions matter

## Monitoring the Experiment

### Watch Gazebo
- Observe robot movement in real-time
- Note when behavior changes

### Monitor Topics
```bash
# In another terminal
ros2 topic echo /cmd_vel
```

### Check Nodes
```bash
ros2 node list
ros2 node info /normal_operator
ros2 node info /injection_attacker
```

### View Logs
```bash
# Normal controller logs
cat /tmp/normal_controller.log

# ROS2 logs
./scripts/testing/view_logs.sh
```

## Experiment Variations

### Test Different Frequencies
```bash
# Low frequency attack (may fail)
python3 injection_attack.py --attack-type override --frequency 5 --duration 20

# Very high frequency attack (likely to succeed)
python3 injection_attack.py --attack-type override --frequency 100 --duration 20
```

### Test Different Patterns
```bash
# Normal controller with circle pattern
python3 normal_controller.py --pattern circle --frequency 10 --duration 60

# Then launch attack in another terminal
python3 injection_attack.py --attack-type override --frequency 50 --duration 20
```

### Test Stealth Attack
```bash
# Stealth attack with moderate frequency
python3 injection_attack.py --attack-type stealth --frequency 25 --duration 30
```

## Expected Observations

1. **Before Attack**: Robot follows normal pattern (square/circle/etc.)

2. **During Attack**: 
   - Robot behavior changes
   - May stop following normal pattern
   - Follows attack commands instead

3. **After Attack**:
   - Robot may return to normal pattern
   - Or stop if normal controller finished

## Security Implications

This experiment demonstrates:
- **Vulnerability**: ROS2 topics are open by default
- **No Authentication**: Any node can publish to `/cmd_vel`
- **Frequency-Based Override**: Higher frequency can dominate topic
- **Need for Security**: SROS2 or access control needed for real systems

## Defense Measures

1. **Use SROS2**: Secure ROS2 with encryption and authentication
2. **Access Control**: Limit which nodes can publish to critical topics
3. **Rate Limiting**: Implement rate limiting on control topics
4. **Monitoring**: Monitor for unexpected publishers
5. **Domain Isolation**: Use ROS_DOMAIN_ID to isolate networks

## Troubleshooting

### Attack Not Working
- Check if both nodes are running: `ros2 node list`
- Verify topics: `ros2 topic list | grep cmd_vel`
- Check message flow: `ros2 topic hz /cmd_vel`
- Increase attack frequency

### Normal Controller Not Working
- Check ROS2 environment: `echo $ROS_DISTRO`
- Verify TURTLEBOT3_MODEL: `echo $TURTLEBOT3_MODEL`
- Check Gazebo is running: `ros2 topic list`

### Script Errors
- Ensure scripts are executable: `chmod +x *.sh *.py`
- Check Python dependencies: `python3 -c "import rclpy"`
- Verify ROS2 installation

## Next Steps

After understanding this basic injection attack:
1. Try network-based attacks (from different machine)
2. Experiment with different attack strategies
3. Implement and test defense measures
4. Document findings in your security research report

