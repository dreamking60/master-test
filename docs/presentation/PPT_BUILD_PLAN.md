# PPT Build Plan

This document gives a practical slide-by-slide plan for the final presentation.

## Recommended Length

Use 12 to 15 slides for a standard class presentation.

Target timing:

- 8 minutes: 10 to 12 slides.
- 12 minutes: 12 to 15 slides.
- 15 minutes: 15 slides plus live demo.

## Visual Style

Use a clean technical style:

- Dark text on light background or high-contrast dark theme.
- Use one accent color for attacks, such as red or orange.
- Use one accent color for defenses, such as blue or green.
- Keep commands short and readable.
- Use diagrams instead of paragraphs where possible.

## Slide Deck Outline

### Slide 1: Title

Title:

```text
ROS2 Security Evaluation on TurtleBot3
```

Subtitle:

```text
Command Injection, SROS2 Defense, and MITM Experiment Migration
```

Include:

- Name.
- Course/project.
- Date.

### Slide 2: Motivation

Message:

```text
Robots are cyber-physical systems. A communication attack can become physical motion.
```

Show:

- TurtleBot3/Gazebo screenshot.
- Small diagram: Controller -> ROS2 -> Robot.

Talking points:

- ROS2 connects controllers, sensors, and actuators.
- If command topics are open, attackers can affect physical behavior.

### Slide 3: Research Questions

Use three questions:

1. Can an unauthorized ROS2 node inject motion commands?
2. Can SROS2 block unauthorized command publishers?
3. How should network-layer MITM be tested in the migrated environment?

### Slide 4: Original vs Migrated Environment

Show two diagrams:

Original:

```text
VM Controller | VM Attacker | VM Robot/Gazebo
```

Migrated:

```text
Docker Controller | Docker Attacker | WSL Host Gazebo
```

Important phrase:

```text
Gazebo remains on WSL host for GUI; controller and attacker are containerized.
```

### Slide 5: Stable WSL + Docker Data Path

Diagram:

```text
Controller/Attacker containers
        |
        v
     /cmd_vel_in
        |
        v
  host cmd_vel_relay
        |
        v
      /cmd_vel
        |
        v
  ros_gz_bridge -> Gazebo
```

Explain why relay exists:

- Docker-to-Gazebo DDS discovery was visible but data delivery was unstable.
- Relay gives an observable and stable host-side data path.

### Slide 6: Experiment 1 Setup

Title:

```text
Experiment 1: Open ROS2 Command Injection
```

Table:

| Role | Location | Behavior |
| --- | --- | --- |
| Controller | Docker | forward commands at 10 Hz |
| Attacker | Docker | turn commands at 50 Hz |
| Robot | WSL/Gazebo | executes resulting command stream |

Demo command:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

### Slide 7: Experiment 1 Result

Show:

- Gazebo screenshot or short video.
- Trajectory plot if available.
- Log snippet: `relayed=...`

Main finding:

```text
Without authentication or access control, an attacker can join the ROS2 graph and publish command messages.
```

### Slide 8: Experiment 2 SROS2 Defense Design

Title:

```text
Experiment 2: SROS2 Command Topic Access Control
```

Policy table:

| Enclave | Allowed |
| --- | --- |
| `/controller` | publish `/cmd_vel_in` |
| `/gazebo` | subscribe `/cmd_vel_in`, publish `/cmd_vel` |
| `/attacker` | no command-topic publish permission |

Show file:

```text
config/sros2_wsl_docker_policy.xml
```

### Slide 9: SROS2 Migration Issue and Fix

Problem:

```text
couldn't find all security files
```

Root cause:

```text
Jazzy needed enclave override or explicit --enclave argument.
```

Fix:

```text
ROS_SECURITY_ENCLAVE_OVERRIDE=/controller
--ros-args --enclave /controller
```

This slide is useful because it shows real engineering debugging, not only theory.

### Slide 10: Experiment 2 Demo and Evidence

Demo command:

```bash
./scripts/demo/tmux_sros2_defense_demo.sh
```

Evidence commands:

```bash
./experiments/02_sros2_cmd_vel_defense/validate_policy.sh
./experiments/02_sros2_cmd_vel_defense/collect_evidence.sh
```

Expected result:

- Controller works.
- Attacker cannot affect command path.

### Slide 11: Experiment 3 MITM Motivation

Message:

```text
Publisher injection and network MITM are different attack models.
```

Compare:

| Attack | What attacker does |
| --- | --- |
| Injection | publishes ROS2 messages |
| MITM | controls or observes network path |

Explain:

- ARP spoofing requires separate layer-2 identities.
- Docker host networking cannot represent that correctly.

### Slide 12: Docker Bridge MITM Lab

Show topology:

```text
controller 172.28.0.10
robot      172.28.0.20
attacker   172.28.0.50
```

Demo command:

```bash
./scripts/demo/tmux_network_mitm_demo.sh
```

Mention:

- Uses `docker-compose.mitm.yml`.
- Gives each role separate IP/MAC identity.

### Slide 13: ARP Helper and Safety Controls

Show:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --duration 6
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20
```

Safety constraints:

- Dry-run by default.
- Requires `--execute`.
- Defaults to `172.28.0.0/24`.
- Supports restore.

### Slide 14: Results Summary

Table:

| Experiment | Status | Key Result |
| --- | --- | --- |
| Open injection | Verified | attacker can influence robot command path |
| SROS2 defense | Migrated, policy validated | attacker lacks command publish permission |
| Network MITM | Bridge lab migrated | separate L2 identities available |
| SLAM security | Future work | perception integrity remains to study |

### Slide 15: Conclusion and Future Work

Conclusion:

```text
ROS2 robot security must be evaluated at topic, middleware, network, and perception layers.
```

Future work:

- Complete SROS2 runtime validation.
- Measure ROS2 command traffic during MITM state changes.
- Extend experiments to SLAM topics such as `/scan`, `/odom`, `/tf`, and camera streams.
- Test on real TurtleBot3 hardware.

## Live Demo Strategy

Do not run every demo live unless you have enough time. Recommended live demo:

1. Start with open injection:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

2. Show normal movement.
3. Press Enter in attacker pane.
4. Show trajectory deviation.

If time allows, show the SROS2 policy validation:

```bash
./experiments/02_sros2_cmd_vel_defense/validate_policy.sh
```

For MITM, it may be safer to show the tmux panes and evidence logs instead of live ARP poisoning:

```bash
./scripts/demo/tmux_network_mitm_demo.sh
```

## Figures to Prepare

Prepare these visuals before making the final slides:

1. Architecture before migration.
2. Architecture after WSL + Docker migration.
3. `/cmd_vel_in -> relay -> /cmd_vel` data path.
4. Screenshot of Gazebo robot in empty world.
5. Screenshot of tmux open injection demo.
6. SROS2 permission table.
7. MITM Docker bridge topology.
8. Final experiment summary table.

## Recommended PPT Production Workflow

1. Make diagrams first.
2. Build slides around diagrams.
3. Keep code screenshots short.
4. Use one slide per experiment result.
5. Put backup commands in appendix slides.

## Appendix Slides

Optional backup slides:

- Exact environment variables.
- Key files and scripts.
- Full command list.
- Troubleshooting note about `ROS_SECURITY_ENCLAVE_OVERRIDE`.
- Limitations and next steps.

