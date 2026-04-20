# Presentation and Report Guide

## Core Presentation Logic

The presentation should emphasize experiment design, not only implementation. For each experiment, use the same structure:

```text
Goal -> Hypothesis -> Experiment Design -> Evidence/Measurement -> Result -> Conclusion
```

A useful speaking pattern is:

```text
The goal of this experiment was to verify [security claim].
To test it, I designed [controlled setup].
I measured [specific evidence].
The result was [observed data].
This supports the conclusion that [security meaning].
```

This makes the project sound like a security evaluation rather than a collection of scripts.

## Recommended Presentation Structure

### Slide 1: Project Motivation

Robots are cyber-physical systems. A ROS2 robot depends on networked communication between controllers, sensors, middleware, and actuators. If an attacker can inject or modify these messages, the robot can behave unsafely.

### Slide 2: System Under Test

Show the TurtleBot3 simulation stack:

```text
Controller -> ROS2 DDS -> /cmd_vel -> Gazebo TurtleBot3
```

For the migrated setup, show:

```text
Docker controller/attacker -> /cmd_vel_in -> WSL relay -> /cmd_vel -> Gazebo
```

### Slide 3: Environment Migration

Explain why the project moved from three VMware VMs to WSL + Docker:

- Gazebo needs a visible desktop/GUI.
- Controller and attacker benefit from container isolation.
- WSL + Docker is easier to reproduce than multiple manually maintained VMs.
- DDS discovery and transport in WSL + Docker required a relay to make the data path observable and stable.

### Slide 4: Experiment 01 - Open Injection

Goal:

```text
Verify whether an unauthorized ROS2 node can affect robot motion when SROS2 is disabled.
```

Design:

```text
Controller publishes forward commands.
Attacker publishes competing turn commands.
Gazebo executes the resulting command stream.
```

Measurement:

```text
Robot trajectory, topic publishers, relay logs.
```

Conclusion:

```text
Without authentication or access control, a discovered attacker node can influence robot motion.
```

Show:

- Controller at 10 Hz.
- Attacker at 50 Hz.
- Robot trajectory deviation.
- Topic info showing competing publishers on `/cmd_vel_in`.

Demo command:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

### Slide 5: Experiment 02 - SROS2 Defense

Goal:

```text
Verify whether SROS2 can block unauthorized command publishers while allowing the legitimate controller.
```

Design:

```text
/controller can publish /cmd_vel_in.
/gazebo can subscribe and relay.
/attacker has no command-topic publish permission.
```

Measurement:

```text
Controller behavior, attacker error logs, generated permissions.
```

Conclusion:

```text
SROS2 changes the problem from open publication to authenticated, permission-controlled publication.
```

Show:

- `/controller` can publish `/cmd_vel_in`.
- `/attacker` cannot publish `/cmd_vel_in`.
- `/gazebo` can relay and bridge command traffic.

Demo command:

```bash
./scripts/demo/tmux_sros2_defense_demo.sh
```

### Slide 6: Experiment 03 - MITM

Goal:

```text
Verify whether a network-path attacker can position itself between controller and robot.
```

Design:

```text
Use Docker bridge instead of host networking, so controller, robot, and attacker have independent IP/MAC identities.
```

Measurement:

```text
ARP tables, qdisc state, ping RTT, ROS2 command latency.
```

Conclusion:

```text
Network-path attacks require a different topology from open publisher injection.
```

Explain:

- ARP spoofing needs separate L2 identities.
- Docker host network is not appropriate for ARP spoofing.
- Use Docker bridge or multi-VM topology.

Demo command:

```bash
./scripts/demo/tmux_network_mitm_demo.sh
```

The tmux demo does not automatically launch ARP poisoning. For the isolated bridge lab only, use:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20
```

### Slide 7: SROS2 DoS / Availability

Goal:

```text
Verify whether SROS2-secured command traffic can still suffer availability degradation.
```

Design:

```text
Run secure controller and secure Gazebo, then flood discovered DDS/RTPS UDP ports without becoming a valid publisher.
```

Measurement:

```text
Packet count, attacker CPU, controller stall, relay watchdog.
```

Conclusion:

```text
SROS2 blocks unauthorized command injection, but availability still needs separate protection.
```

Show:

- DDS/RTPS UDP flood targeted discovered ports `7400, 7410-7423`.
- Attacker sent `613,326` packets in `26.2s`.
- Controller showed about 3 seconds of publish-loop stall.
- Relay logged `No /cmd_vel_in for 2.93s` and sent zero `/cmd_vel`.

Reference:

```text
docs/presentation/DOS_QA_CHEATSHEET.md
docs/presentation/SECURITY_PRE_QA_CHEATSHEET.md
```

### Slide 8: Future Work - SLAM Security

Connect command-channel security to perception security:

- Protecting `/cmd_vel` is not enough.
- `/scan`, `/odom`, `/tf`, camera, and map topics affect localization and navigation.
- ORB-SLAM3 or TurtleBot3 SLAM experiments can evaluate perception-level attacks.

## Suggested Result Table

| Experiment | Goal | Design | Verified Result | Conclusion |
| --- | --- | --- | --- | --- |
| 01 Open Injection | test default ROS2 exposure | attacker publishes competing command topic | robot deviates | open ROS2 command path is unsafe |
| 02 SROS2 Defense | test authorization | attacker lacks command permission | publisher blocked | SROS2 blocks unauthorized command injection |
| 03 SROS2 DoS | test availability boundary | flood DDS/RTPS ports | stalls/watchdog events | SROS2 is not full availability defense |
| 04 Open MITM | test network-path control | ARP poison bridge endpoints | latency/gap increases, tamper possible in open mode | MITM topology reproduced |
| 05 SROS2 MITM | test protected payload under MITM | ARP MITM + encrypted DDS | delay/drop but no valid rewrite | SROS2 protects integrity, not availability |
| 06 SLAM Future Work | test perception integrity | corrupt `/scan`, `/odom`, `/tf`, camera/map | not implemented yet | future security layer |

## Writing Advice

Use this structure in the final report:

1. Environment and reproducibility.
2. Baseline vulnerability.
3. Defense mechanism.
4. Network-layer extension.
5. Limitations and future work.

Be precise about what has been verified and what is still planned. For example, the open injection and SROS2 access-control experiments are verified in WSL + Docker, while trajectory CSV quantification for the SROS2 DoS run still needs a recorder launched with the correct security enclave.

## Suggested Figures

- Architecture before migration: three VM layout.
- Architecture after migration: WSL Gazebo plus Docker controller/attacker.
- Data path diagram: `/cmd_vel_in -> relay -> /cmd_vel`.
- Trajectory plot: normal vs attacked.
- SROS2 permission diagram: controller allowed, attacker denied.
- DoS evidence: DDS ports, packet count, controller stall, relay watchdog.
- MITM topology: controller, attacker router, robot.
- SROS2 MITM evidence: ARP poisoning succeeded, `tc netem` delay/loss applied, SROS2 command latency/gaps increased, but command payload was not validly rewritten.

## Key Terms

- Open ROS2 environment: ROS2 graph without SROS2 authentication or access control.
- Unauthorized command injection: an attacker publishes robot command messages without permission.
- Publisher competition: multiple publishers write to the same command path, and effective behavior can be dominated by rate/timing.
- SROS2 enclave: security identity and permission boundary for ROS2 nodes.
- DDS/RTPS UDP flood: availability test that stresses ROS2 middleware ports without becoming an authorized command publisher.
- MITM: attacker controls or observes the network path between legitimate endpoints.
- Availability degradation: the robot communication path remains protected from unauthorized commands, but packet delay/loss creates timing gaps or watchdog stop events.
