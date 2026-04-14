# Presentation and Report Guide

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

Claim:

```text
Without SROS2, any discovered node can publish robot motion commands.
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

Claim:

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

Claim:

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

### Slide 7: Future Work - SLAM Security

Connect command-channel security to perception security:

- Protecting `/cmd_vel` is not enough.
- `/scan`, `/odom`, `/tf`, camera, and map topics affect localization and navigation.
- ORB-SLAM3 or TurtleBot3 SLAM experiments can evaluate perception-level attacks.

## Suggested Result Table

| Experiment | Security Mode | Attack Method | Expected Result | Evidence |
| --- | --- | --- | --- | --- |
| 01 | SROS2 off | extra publisher on command path | robot deviates | trajectory, logs, topic info |
| 02 | SROS2 on | attacker without command permission | attacker blocked | permissions XML, topic info, robot behavior |
| 03 | SROS2 off/on | network-path MITM | depends on topology/security | ARP cache, packet capture, command log |
| 04 | TBD | SLAM input corruption | mapping/localization degradation | map quality, drift metrics |

## Writing Advice

Use this structure in the final report:

1. Environment and reproducibility.
2. Baseline vulnerability.
3. Defense mechanism.
4. Network-layer extension.
5. Limitations and future work.

Be precise about what has been verified and what is still planned. For example, the open injection experiment is verified in WSL + Docker, while the SROS2 policy is implemented but runtime validation is still being debugged.

## Suggested Figures

- Architecture before migration: three VM layout.
- Architecture after migration: WSL Gazebo plus Docker controller/attacker.
- Data path diagram: `/cmd_vel_in -> relay -> /cmd_vel`.
- Trajectory plot: normal vs attacked.
- SROS2 permission diagram: controller allowed, attacker denied.
- MITM topology: controller, attacker router, robot.

## Key Terms

- Open ROS2 environment: ROS2 graph without SROS2 authentication or access control.
- Unauthorized command injection: an attacker publishes robot command messages without permission.
- Publisher competition: multiple publishers write to the same command path, and effective behavior can be dominated by rate/timing.
- SROS2 enclave: security identity and permission boundary for ROS2 nodes.
- MITM: attacker controls or observes the network path between legitimate endpoints.
