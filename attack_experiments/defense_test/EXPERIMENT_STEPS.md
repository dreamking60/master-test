# Defense Mechanism Testing - Step by Step

## Overview

After successfully demonstrating network attacks, we now test various defense mechanisms to see which ones are effective.

## Defense Mechanisms to Test

1. **ROS_DOMAIN_ID Isolation** - Basic network isolation
2. **Firewall Rules** - Block DDS ports
3. **Node Monitoring** - Detect unexpected publishers
4. **Rate Limiting** - Detect high-frequency attacks
5. **ARP Integrity Monitoring** - Detect suspicious ARP/neighbor table changes (MITM symptom)

---

## Test 1: ROS_DOMAIN_ID Isolation

### Purpose
Test if using different ROS_DOMAIN_IDs prevents network attacks.

### Steps

1. **On target machine** (current machine):
   ```bash
   # Start Gazebo with default domain ID
   export ROS_DOMAIN_ID=0
   cd /home/stevenchen/master-test
   ./scripts/setup/run_empty_world.sh
   ```

2. **On attacker machine**:
   ```bash
   cd defense_test/scripts
   ./test_domain_isolation.sh
   ```

3. **Follow the prompts**:
   - Enter target IP
   - Test with same domain ID (should work)
   - Test with different domain ID (should fail)

### Expected Result
- Same domain ID: Attack works ✅
- Different domain ID: Attack fails ✅

### Conclusion
ROS_DOMAIN_ID provides basic isolation but is NOT secure (attacker can guess).

---

## Test 2: Firewall Protection

### Purpose
Test if blocking DDS ports prevents network attacks.

### Steps

1. **On target machine**:
   ```bash
   cd defense_test/scripts
   ./test_firewall.sh
   ```

2. **Follow the prompts** to block DDS ports (7400-7500/udp)

3. **On attacker machine**, try to discover target:
   ```bash
   export ROS_DOMAIN_ID=0
   ros2 topic list  # Should NOT see /cmd_vel
   ```

4. **Restore firewall** after testing

### Expected Result
- With firewall: Attack fails ✅
- Without firewall: Attack works ✅

### Conclusion
Firewall is very effective but may block legitimate remote access.

---

## Test 3: Node Monitoring

### Purpose
Detect unexpected publishers on critical topics.

### Steps

1. **On target machine**, start monitoring:
   ```bash
   cd defense_test/scripts
   ./test_node_monitoring.sh
   ```

2. **In another terminal**, start normal controller:
   ```bash
   cd /home/stevenchen/master-test/attack_experiments/scripts
   python3 normal_controller.py --pattern continuous --frequency 10
   ```

3. **From attacker machine**, launch attack:
   ```bash
   python3 injection_attack.py --attack-type turn_left --frequency 50
   ```

4. **Observe monitoring output** - should detect the attacker node

### Expected Result
Monitoring detects the attacker node name (e.g., "injection_attacker").

### Conclusion
Node monitoring can detect attacks but requires active monitoring.

---

## Test 4: Rate Limiting

### Purpose
Detect high-frequency attacks by monitoring message rates.

### Steps

1. **On target machine**, start rate monitoring:
   ```bash
   cd defense_test/scripts
   ./test_rate_limiting.sh
   ```

2. **In another terminal**, start normal controller (10 Hz):
   ```bash
   python3 normal_controller.py --pattern continuous --frequency 10
   ```

3. **From attacker machine**, launch high-frequency attack (50 Hz):
   ```bash
   python3 injection_attack.py --attack-type turn_left --frequency 50
   ```

4. **Observe rate monitoring** - should detect high rate

### Expected Result
Monitoring detects rate spike from 10 Hz to 50+ Hz.

### Conclusion
Rate limiting can detect attacks but requires threshold tuning.

---

## Test 5: ARP Integrity Monitoring (MITM Symptom Detection)

### Purpose
Detect suspicious ARP/neighbor table changes that often accompany man-in-the-middle positioning attempts, especially when the **default gateway IP** suddenly resolves to a different MAC address.

This is a **defensive** test: it collects evidence (baseline + logs) and alerts on IP→MAC changes.

### Steps

1. **On the machine you want to protect/observe** (robot, controller, or router VM), start ARP monitoring:

   ```bash
   cd defense_test/scripts
   bash ./test_arp_monitoring.sh
   ```

2. The script will:
   - Detect your default gateway and interface
   - Take a baseline neighbor table snapshot
   - Watch the gateway IP (and any extra IPs you enter) for MAC changes
   - Write logs and a baseline JSON under `defense_test/results/`

3. **While the monitor is running**, perform your networking experiments (e.g., any MITM routing/gateway changes or lab manipulations). If an IP→MAC mapping changes, you should see an `ALERT` line in the monitor output and log.

### Expected Result
- Normal network: gateway IP mapping stays stable → mostly `INFO`/`NOTICE`
- If neighbor mapping changes: `ALERT <ip>: MAC changed <old> -> <new> (baseline was <base>)`

### Notes / Interpretation
- **Gateway MAC change** is a high-signal indicator that the host's L2 path may have been altered.
- **MAC collisions** (multiple IPs mapping to one MAC) are a weak signal; they can happen legitimately (NAT/bridges), but are useful context.

---

## Advanced: Combined Defenses

Test multiple defenses together:

1. **ROS_DOMAIN_ID + Firewall**:
   - Use non-default domain ID
   - Block DDS ports
   - Very strong defense

2. **Monitoring + Rate Limiting**:
   - Monitor for unexpected nodes
   - Alert on high message rates
   - Good for detection

3. **All Defenses Combined**:
   - Domain isolation
   - Firewall rules
   - Active monitoring
   - Rate limiting
   - Maximum security

---

## Defense Effectiveness Summary

| Defense | Effectiveness | Ease of Implementation | Notes |
|---------|-------------|------------------------|-------|
| ROS_DOMAIN_ID | ⭐⭐ (2/5) | ⭐⭐⭐⭐⭐ (5/5) | Easy but not secure |
| Firewall | ⭐⭐⭐⭐ (4/5) | ⭐⭐⭐⭐ (4/5) | Very effective |
| Node Monitoring | ⭐⭐⭐ (3/5) | ⭐⭐⭐ (3/5) | Requires active monitoring |
| Rate Limiting | ⭐⭐⭐ (3/5) | ⭐⭐⭐ (3/5) | Requires tuning |

## Best Practices

1. **Use multiple defenses** - Don't rely on a single mechanism
2. **Use non-default ROS_DOMAIN_ID** - Easy first step
3. **Configure firewall** - Block DDS ports if remote access not needed
4. **Monitor critical topics** - Detect attacks early
5. **Use SROS2 for production** - Proper encryption and authentication

---

## Next Steps

After testing defenses, you can:
1. Document which defenses work best
2. Create a defense configuration guide
3. Test on real robots
4. Implement automated monitoring

