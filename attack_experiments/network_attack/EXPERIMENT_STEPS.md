# Network Attack Experiment - Step by Step Guide

## Prerequisites

You need:
- **Machine 1 (Target Machine)**: Current machine, running Gazebo and robot
- **Machine 2 (Attacker Machine)**: Another machine, running attack scripts
- Both machines on the same network, or can access each other

---

## Step 1: Setup on Target Machine (Current Machine)

### 1.1 Open terminal and navigate to experiment directory

```bash
cd /home/stevenchen/master-test/attack_experiments/network_attack/scripts
```

### 1.2 Run target machine setup script

```bash
./setup_target_machine.sh
```

### 1.3 Follow the prompts

The script will:
1. Automatically get your machine's IP address
2. Ask for ROS_DOMAIN_ID (press Enter to use default value 0)
3. Save configuration information
4. **Automatically start Gazebo**

### 1.4 Record important information

The script will display:
```
Target machine information:
  IP Address: 192.168.x.x    ← Note this IP!
  ROS_DOMAIN_ID: 0           ← Note this!
```

**Important**: Share this IP address and Domain ID with the attacker machine!

### 1.5 Keep Gazebo running

- Gazebo will start automatically
- You'll see the robot on an empty plane
- **Do not close this terminal**, keep Gazebo running

---

## Step 2: Setup on Attacker Machine (Another Machine)

### 2.1 On another machine, copy experiment files

You need to copy the `network_attack` folder to the attacker machine, or directly on the attacker machine:

```bash
# On attacker machine, navigate to project directory
cd /path/to/master-test/attack_experiments/network_attack/scripts
```

### 2.2 Run attacker machine setup script

```bash
./setup_attacker_machine.sh
```

### 2.3 Enter target machine information

The script will ask:
1. **Target machine IP address**: Enter the IP address recorded in step 1.4
2. **Target machine ROS_DOMAIN_ID**: Enter the Domain ID recorded in step 1.4 (usually 0)

### 2.4 Wait for configuration to complete

The script will:
- Test network connection (ping target machine)
- Check if on the same subnet
- Create DDS routing configuration if needed
- Restart ROS2 daemon
- Test node discovery

### 2.5 Verify connection

If you see:
```
/cmd_vel topic discovered - ready to attack!
```

Connection successful! You can continue.

If you see:
```
/cmd_vel topic not found
```

Check:
- Is Gazebo running on the target machine?
- Is the IP address correct?
- Is the network connected?

---

## Step 3: Launch Attack

### 3.1 On attacker machine, choose attack method

**Method A: Interactive Attack (Recommended)**

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./launch_attack_interactive.sh
```

The script will display a menu to select attack type:
```
Select attack type:
  1) Turn Left - Forces robot to turn left
  2) Override - Forces robot to move forward
  3) Spin - Makes robot spin rapidly
  4) Stealth - Lower frequency attack
  5) Interference - Random commands
  6) Custom - Set all parameters manually
```

**Method B: Quick Attack (Command-line arguments)**

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./quick_attack.sh turn_left 50 15
# Parameters: attack_type frequency duration
# Or use defaults: ./quick_attack.sh
```

**Method C: Original Script**

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./launch_attack.sh
```

### 3.2 Observe attack effects

- Observe in the **target machine's** Gazebo window
- Robot should start turning left (if turn_left was selected)
- Attack will last for the specified duration (default 15 seconds)

---

## Quick Command Summary

### Target Machine (Current Machine)

```bash
# Terminal 1: Setup and start
cd /home/stevenchen/master-test/attack_experiments/network_attack/scripts
./setup_target_machine.sh
# Record the displayed IP and Domain ID
# Gazebo will start automatically, keep it running
```

### Attacker Machine (Another Machine)

```bash
# Terminal 1: Setup attacker machine
cd /path/to/attack_experiments/network_attack/scripts
./setup_attacker_machine.sh
# Enter target machine's IP and Domain ID

# Terminal 2: Launch attack
cd /path/to/attack_experiments/network_attack/scripts
./launch_attack.sh
# Select attack type
```

---

## Manual Attack (Optional)

If you want to manually control attack parameters:

```bash
# On attacker machine
cd /path/to/attack_experiments/network_attack/scripts

# Ensure environment variables are set (setup_attacker_machine.sh sets them)
export ROS_DOMAIN_ID=0  # Same as target machine

# Run attack
python3 ../../scripts/injection_attack.py \
    --attack-type turn_left \
    --frequency 50 \
    --duration 15 \
    --angular-speed 0.5
```

---

## Commands to Verify Connection

### Verify on attacker machine

```bash
# Check if target nodes can be discovered
ros2 node list

# Check if /cmd_vel topic can be discovered
ros2 topic list | grep cmd_vel

# View /cmd_vel messages in real-time
ros2 topic echo /cmd_vel

# Check message frequency
ros2 topic hz /cmd_vel
```

---

## Common Issues

### Q: Attacker machine cannot find target machine

**Checklist**:
1. ✅ Is Gazebo running on the target machine?
2. ✅ Is the IP address correct?
3. ✅ Is ROS_DOMAIN_ID the same?
4. ✅ Is the network connected? Try `ping <target_ip>`
5. ✅ Is firewall blocking DDS ports?

**Solution**:
```bash
# On attacker machine, restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
sleep 5
ros2 topic list
```

### Q: Attack has no effect

**Checklist**:
1. ✅ Does `/cmd_vel` topic exist?
2. ✅ Is the attack script running?
3. ✅ Is attack frequency high enough (recommend 50 Hz)?

**Solution**:
```bash
# Check on attacker machine
ros2 topic echo /cmd_vel  # Should see messages being sent

# Check on target machine
ros2 topic echo /cmd_vel  # Should see the same messages
```

### Q: Two machines not on the same subnet

The script will automatically detect and create DDS routing configuration. Ensure:
- `dds_attack_config.xml` file is created
- `CYCLONEDDS_URI` environment variable is set
- ROS2 daemon is restarted

---

## Cleanup After Experiment

### Stop Gazebo (Target Machine)

Press `Ctrl+C` in the terminal running Gazebo

### Clean up configuration (Optional)

```bash
# Delete saved configuration
rm target_ip.txt target_domain_id.txt
rm dds_attack_config.xml  # If exists
```

---

## Next Steps

After successful experiment, you can:
1. Try different attack types
2. Test different attack frequencies
3. Try cross-subnet attacks
4. Test defense measures (change ROS_DOMAIN_ID)
