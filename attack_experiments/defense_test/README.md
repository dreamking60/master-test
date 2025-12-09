# Defense Mechanism Testing

This experiment tests the effectiveness of various defense mechanisms against ROS2 command injection attacks.

## Defense Mechanisms to Test

1. **ROS_DOMAIN_ID Isolation** - Using different domain IDs to isolate networks
2. **Firewall Rules** - Blocking DDS ports to prevent network attacks
3. **Node Monitoring** - Detecting unexpected publishers on critical topics
4. **Message Rate Limiting** - Limiting message frequency to prevent high-frequency attacks
5. **Topic Access Control** - Restricting which nodes can publish to critical topics

## Quick Start

### Test 1: ROS_DOMAIN_ID Isolation

```bash
cd defense_test/scripts
./test_domain_isolation.sh
```

### Test 2: Firewall Protection

```bash
cd defense_test/scripts
./test_firewall.sh
```

### Test 3: Node Monitoring

```bash
cd defense_test/scripts
./test_node_monitoring.sh
```

### Test 4: Rate Limiting

```bash
cd defense_test/scripts
./test_rate_limiting.sh
```

## Experiment Structure

```
defense_test/
├── README.md                    # This file
├── EXPERIMENT_STEPS.md          # Detailed steps
├── scripts/
│   ├── test_domain_isolation.sh    # Test ROS_DOMAIN_ID isolation
│   ├── test_firewall.sh            # Test firewall rules
│   ├── test_node_monitoring.sh     # Test node monitoring
│   ├── test_rate_limiting.sh       # Test rate limiting
│   └── monitor_nodes.py            # Node monitoring tool
└── docs/
    └── DEFENSE_RESULTS.md          # Results documentation
```

## Expected Results

After running defense tests, you should understand:
- Which defenses are effective
- Which defenses are easy to bypass
- Best practices for securing ROS2 systems
- Trade-offs between security and usability

