# Fix SROS2 Security Configuration

## Problem

When using SROS2 security mode, you may see this error:
```
SECURITY ERROR: directory '/path/to/keys/node_name/enclaves' does not exist.
```

This happens because SROS2 requires an `enclaves` directory in the keystore, but our initialization script didn't create it.

## Quick Fix

### Option 1: Run Fix Script (Easiest)

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts
./fix_enclaves.sh
```

This will automatically:
- Find all node directories
- Create missing `enclaves` directories
- Use `ros2 security` commands to properly initialize them

### Option 2: Re-run Initialization

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts
./init_sros2_security.sh
```

The updated script now automatically creates `enclaves` directories.

### Option 3: Manual Fix

For a specific node:

```bash
# Set the node directory
NODE_DIR="/home/stevenchen/master-test/attack_experiments/sros2_test/keys/robot_controller"

# Create enclaves directory
mkdir -p "$NODE_DIR/enclaves"

# Create keystore and enclave
ros2 security create_keystore "$NODE_DIR"
ros2 security create_enclave "$NODE_DIR" "/"
```

## Verification

After fixing, verify the directory exists:

```bash
ls -la /home/stevenchen/master-test/attack_experiments/sros2_test/keys/robot_controller/enclaves
```

You should see the `enclaves` directory.

## Using Security Mode

After fixing, you can use security mode:

```bash
# Enable security
cd /home/stevenchen/master-test/attack_experiments/sros2_test
source setup_robot_controller_env.sh

# Start Gazebo (should work now)
cd ../..
./scripts/setup/run_empty_world.sh
```

The robot should spawn correctly with security enabled!

## What Was Fixed

1. **init_sros2_security.sh** - Now creates `enclaves` directories automatically
2. **All environment scripts** - Now check and create `enclaves` if missing
3. **run_empty_world.sh** - Now creates `enclaves` if security is enabled but missing
4. **fix_enclaves.sh** - New script to fix existing configurations

## Notes

- The `enclaves` directory is required by SROS2
- It's created automatically when you use the environment scripts
- If you see the error, just run `fix_enclaves.sh` to fix it

