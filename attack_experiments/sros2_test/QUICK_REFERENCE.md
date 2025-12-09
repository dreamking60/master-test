# SROS2 Security Quick Reference

## Setup Complete! ✅

All certificates and policies have been created.

## How to Use

### Method 1: Use Node-Specific Script

```bash
source setup_robot_controller_env.sh
cd ../..
./scripts/setup/run_empty_world.sh
```

### Method 2: Use Generic Script

```bash
source setup_sros2_env.sh robot_controller
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

## Available Nodes

- `robot_controller`
- `normal_operator`
- `gazebo_sim`

## Testing Security

### Test 1: Legitimate Access (Should Work)

```bash
# Terminal 1: Start Gazebo with security
source setup_robot_controller_env.sh
cd ../..
./scripts/setup/run_empty_world.sh

# Terminal 2: Run controller with security
source setup_robot_controller_env.sh
python3 normal_controller.py
```

### Test 2: Unauthorized Access (Should Fail)

```bash
# Terminal 3: Try attack without credentials
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
python3 injection_attack.py --attack-type turn_left
# Should fail - cannot discover nodes
```

## Files Created

- `keys/ca/` - Certificate Authority
- `keys/<node_name>/` - Node certificates and policies
- `setup_<node_name>_env.sh` - Per-node environment scripts
- `setup_sros2_env.sh` - Generic environment script

## Important Notes

- Keep CA private key (`keys/ca/ca_key.pem`) secure!
- All nodes using same CA can communicate
- Policies control what each node can do
- Security is enforced when `ROS_SECURITY_STRATEGY=Enforce`
