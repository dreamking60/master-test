# SROS2 Security - Quick Start Guide

## One-Click Initialization

Run the initialization script to automatically complete all configuration:

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts
./init_sros2_security.sh
```

This script will automatically:
1. ✅ Create Certificate Authority (CA)
2. ✅ Generate node certificates (robot_controller, normal_operator, gazebo_sim)
3. ✅ Create security policies
4. ✅ Generate environment setup scripts

## Using Secure Communication

### Method 1: Use Node-Specific Script (Recommended)

```bash
# Enable security environment for robot_controller
source setup_robot_controller_env.sh

# Then start Gazebo with empty world
cd ../..
./scripts/setup/run_empty_world.sh
```

### Method 2: Use Generic Script

```bash
# Specify node name
source setup_sros2_env.sh robot_controller

# Then start Gazebo with empty world
cd ../..
./scripts/setup/run_empty_world.sh
```

### Method 3: Manual Setup

```bash
export ROS_SECURITY_KEYSTORE="$(pwd)/keys/robot_controller"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

cd ../..
./scripts/setup/run_empty_world.sh
```

## Complete Experiment Flow

### Terminal 1: Start Gazebo (with security)

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test
source setup_robot_controller_env.sh
cd ../..
./scripts/setup/run_empty_world.sh
```

### Terminal 2: Run Normal Controller (with security)

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test
source setup_robot_controller_env.sh
cd ../scripts
python3 normal_controller.py --pattern continuous --frequency 10
```

### Terminal 3: Attempt Attack (no credentials, should fail)

```bash
# Don't set any security credentials
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE

# Attempt attack
cd /home/stevenchen/master-test/attack_experiments/scripts
python3 injection_attack.py --attack-type turn_left --frequency 50
```

**Expected Result**: Attack fails, cannot discover nodes or publish messages.

**View Attack Logs**: See [HOW_TO_VIEW_ATTACK_LOGS.md](HOW_TO_VIEW_ATTACK_LOGS.md) for detailed instructions on viewing attack logs in ROS2 log files.

## Verify Security is Active

### Check Environment Variables

```bash
echo $ROS_SECURITY_KEYSTORE
echo $ROS_SECURITY_ENABLE
echo $ROS_SECURITY_STRATEGY
```

Should show:
- `ROS_SECURITY_KEYSTORE`: Points to node certificate directory
- `ROS_SECURITY_ENABLE`: `true`
- `ROS_SECURITY_STRATEGY`: `Enforce`

### Check Certificates

```bash
ls -la keys/robot_controller/
```

Should see:
- `cert.pem` - Node certificate
- `key.pem` - Node private key
- `ca_cert.pem` - CA certificate
- `policy.xml` - Security policy

### Test Connection

```bash
# Enable security
source setup_robot_controller_env.sh

# Check nodes
ros2 node list

# Check topics
ros2 topic list
```

## Common Issues

### Q: Initialization failed?

**Check**:
1. Is OpenSSL installed: `openssl version`
2. Is ROS2 available: `echo $ROS_DISTRO`

### Q: Security not working?

**Check**:
1. Are environment variables set correctly
2. Do certificate files exist
3. Does policy file exist
4. Are all nodes using the same CA

### Q: Attack still succeeds?

**Possible reasons**:
1. Target node doesn't have security enabled
2. Attacker is using legitimate certificate
3. Security policy misconfigured

## File Structure

```
sros2_test/
├── keys/
│   ├── ca/                    # Certificate Authority
│   │   ├── ca_key.pem        # CA private key (keep secret!)
│   │   └── ca_cert.pem       # CA certificate
│   ├── robot_controller/     # Robot controller certificates
│   │   ├── cert.pem
│   │   ├── key.pem
│   │   ├── ca_cert.pem
│   │   └── policy.xml
│   └── normal_operator/       # Normal operator certificates
│       └── ...
├── setup_robot_controller_env.sh  # Node environment script
├── setup_sros2_env.sh             # Generic environment script
└── QUICK_REFERENCE.md             # Quick reference
```

## Next Steps

1. ✅ Run initialization script
2. ✅ Test legitimate access (should succeed)
3. ✅ Test unauthorized attack (should fail)
4. ✅ Analyze security mechanism effectiveness

