# SROS2 Security Testing - Step by Step

## Overview

This experiment tests ROS2's security mechanism (SROS2) and attempts to attack secured systems to verify if security actually works.

## Prerequisites

1. **ROS2 with SROS2 support**
   ```bash
   # Check if SROS2 is available
   ros2 security --help
   ```

2. **OpenSSL** (for certificate generation)
   ```bash
   sudo apt install openssl
   ```

---

## Step 1: Setup Certificate Authority (CA)

The CA is the root of trust - it signs all node certificates.

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts
./setup_ca.sh
```

This creates:
- `keys/ca/ca_key.pem` - CA private key (keep secure!)
- `keys/ca/ca_cert.pem` - CA certificate

---

## Step 2: Generate Node Certificates

Generate certificates for each node that needs to communicate.

### For Robot Controller (Legitimate Node)

```bash
./generate_certificates.sh
# Enter node name: robot_controller
```

### For Attacker Node (Unauthorized)

```bash
./generate_certificates.sh
# Enter node name: attacker
# But DON'T add it to the security policy!
```

This creates certificates in `keys/robot_controller/` and `keys/attacker/`.

---

## Step 3: Setup Security Policies

Create policies that define what each node can do.

```bash
./setup_security_policies.sh
# Enter node name: robot_controller
```

This creates `keys/robot_controller/policy.xml` that:
- Allows `robot_controller` to publish to `/cmd_vel`
- Does NOT allow `attacker` to publish to `/cmd_vel`

---

## Step 4: Test Secure Communication (Legitimate)

### Terminal 1: Start Gazebo with Security

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts

# Load security environment
export ROS_SECURITY_KEYSTORE="../keys/robot_controller"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Start Gazebo
cd ../../..
./scripts/setup/run_empty_world.sh
```

### Terminal 2: Run Legitimate Controller

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts

# Load security environment (same as Gazebo)
export ROS_SECURITY_KEYSTORE="../keys/robot_controller"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Run controller
cd ../../scripts
python3 normal_controller.py --pattern continuous --frequency 10
```

**Expected**: Robot should move normally. Legitimate access works.

---

## Step 5: Test Unauthorized Attack

### Terminal 3: Attempt Attack WITHOUT Credentials

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts
./test_unauthorized_attack.sh
```

Or manually:

```bash
# NO security credentials
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE

# Try to attack
cd ../../scripts
python3 injection_attack.py --attack-type turn_left --frequency 50
```

**Expected**: Attack should FAIL. Cannot discover nodes or publish messages.

---

## Step 6: Test Attack WITH Wrong Credentials

### Terminal 3: Attempt Attack with Attacker Certificate

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts

# Use attacker certificate (not in policy)
export ROS_SECURITY_KEYSTORE="../keys/attacker"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Try to attack
cd ../../scripts
python3 injection_attack.py --attack-type turn_left --frequency 50
```

**Expected**: Attack should FAIL. Attacker has certificate but no permission in policy.

---

## Step 7: Test Attack WITH Correct Credentials (Bypass Test)

### Terminal 3: Attack with Legitimate Credentials

```bash
cd /home/stevenchen/master-test/attack_experiments/sros2_test/scripts

# Use legitimate certificate (in policy)
export ROS_SECURITY_KEYSTORE="../keys/robot_controller"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Try to attack
cd ../../scripts
python3 injection_attack.py --attack-type turn_left --frequency 50
```

**Expected**: Attack might SUCCEED! 
- If attacker has legitimate credentials, security cannot prevent it
- This shows the importance of protecting certificates

---

## Results Analysis

### Scenario 1: Attack Without Credentials

- **Result**: Should FAIL ✅
- **Conclusion**: SROS2 prevents unauthorized access
- **Security**: Working correctly

### Scenario 2: Attack With Wrong Credentials

- **Result**: Should FAIL ✅
- **Conclusion**: Security policies enforce access control
- **Security**: Working correctly

### Scenario 3: Attack With Stolen Credentials

- **Result**: Might SUCCEED ⚠️
- **Conclusion**: If attacker steals legitimate credentials, security fails
- **Security**: Depends on certificate protection

---

## Key Insights

### What SROS2 Protects Against

1. ✅ **Unauthorized nodes** - Cannot join without certificate
2. ✅ **Network sniffing** - Messages are encrypted
3. ✅ **Message tampering** - Messages are signed
4. ✅ **Policy violations** - Access control enforced

### What SROS2 Does NOT Protect Against

1. ❌ **Stolen credentials** - If attacker gets legitimate certificate
2. ❌ **Compromised nodes** - If legitimate node is hacked
3. ❌ **Insider attacks** - If authorized user is malicious
4. ❌ **CA compromise** - If CA private key is stolen

---

## Security Best Practices

1. **Protect CA private key** - Store securely, never share
2. **Use strong certificates** - Long expiration, proper key sizes
3. **Minimal permissions** - Only grant necessary access
4. **Regular rotation** - Renew certificates periodically
5. **Monitor access** - Log and monitor certificate usage
6. **Network isolation** - Combine with firewall rules

---

## Troubleshooting

### Security Not Working

1. Check if SROS2 is installed:
   ```bash
   ros2 security --help
   ```

2. Verify environment variables:
   ```bash
   echo $ROS_SECURITY_KEYSTORE
   echo $ROS_SECURITY_ENABLE
   ```

3. Check certificate files exist:
   ```bash
   ls -la $ROS_SECURITY_KEYSTORE/
   ```

4. Verify policy file:
   ```bash
   cat $ROS_SECURITY_KEYSTORE/policy.xml
   ```

### Attack Succeeds When It Shouldn't

1. Check if target is actually using SROS2
2. Verify security strategy is "Enforce" (not "Permissive")
3. Check policy file allows/disallows correct nodes
4. Ensure certificates are from same CA

---

## Next Steps

After testing SROS2:

1. **Document results** - Which attacks were blocked?
2. **Test edge cases** - What if CA is compromised?
3. **Performance impact** - Measure encryption overhead
4. **Compare with other defenses** - SROS2 vs firewall vs monitoring

