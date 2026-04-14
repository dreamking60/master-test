# Final Report Draft: ROS2 Security Evaluation on TurtleBot3

Steven Chen

## 1. Introduction

Cyber-physical security is a critical requirement for modern robotic systems. A mobile robot such as TurtleBot3 depends on networked communication between controllers, sensors, middleware, and actuators. If an attacker can join the robot communication graph, inject motion commands, alter network traffic, or corrupt perception data, the result can be unsafe physical behavior rather than only a software failure.

This project evaluates ROS2 security risks and defenses using a TurtleBot3/Gazebo simulation environment. The work focuses on the robot command path, especially velocity control through `/cmd_vel`, and studies how unsecured ROS2 communication can be attacked and how SROS2 can reduce those risks.

The original experimental environment used multiple virtual machines. During the project, the environment was migrated to a more maintainable hybrid WSL2 + Docker architecture:

- Gazebo and the robot visualization run on the WSL host.
- The controller role runs in Docker.
- The attacker role runs in Docker.
- A host relay bridges container command input to the Gazebo command topic.

This migration keeps the visual simulation accessible while preserving role separation for security experiments.

## 2. Project Goals

The project has four main goals:

1. Reproduce command injection attacks against an unsecured ROS2 TurtleBot3 system.
2. Evaluate SROS2 as an authentication and access-control defense.
3. Prepare a network-layer MITM experiment using an isolated Docker bridge topology.
4. Define future work for SLAM and perception-data security.

The project is not only an attack demonstration. The larger objective is to build a repeatable research workflow that can compare insecure and secure robot configurations.

## 3. System Architecture

### 3.1 Original Direction

The early project plan used multiple virtual machines:

- Robot/Gazebo machine.
- Controller machine.
- Attacker machine.

This topology is conceptually clean for security experiments, but it is expensive to maintain. Multiple VMs require repeated ROS2 setup, network configuration, GUI support, and dependency management.

### 3.2 Migrated WSL + Docker Architecture

The migrated architecture is:

```text
Docker controller/attacker
  publish /cmd_vel_in
        |
        v
WSL host cmd_vel_relay
  subscribe /cmd_vel_in
  publish /cmd_vel
        |
        v
ros_gz_bridge
        |
        v
Gazebo TurtleBot3
```

This architecture was chosen because Gazebo needs reliable GUI support, while controller and attacker nodes benefit from container isolation. The key engineering change is the host-side relay:

- Containers publish to `/cmd_vel_in`.
- The host relay republishes to `/cmd_vel`.
- Gazebo consumes `/cmd_vel` through the ROS-Gazebo bridge.

The relay was introduced because direct DDS communication from Docker containers to Gazebo was unreliable in the WSL2 environment even when endpoints were discoverable.

### 3.3 Environment Configuration

The stable configuration uses:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ROS_LOCALHOST_ONLY=0
```

The final scripts are organized under:

- `scripts/setup/`
- `scripts/wsl_docker/`
- `scripts/demo/`
- `experiments/`
- `deployment/wsl_docker/`

To avoid cross-experiment interference, the demo scripts run a shared cleanup step before starting. The cleanup script stops old tmux sessions, Docker containers, Gazebo processes, relay processes, and resets the ROS2 daemon.

## 4. Experiment 1: Open ROS2 `/cmd_vel` Injection

### 4.1 Purpose

This experiment demonstrates that an unsecured ROS2 graph allows an unauthorized node to publish robot motion commands.

### 4.2 Threat Model

The attacker can join the same ROS2 domain as the controller and robot. SROS2 is disabled. The attacker does not need a certificate or permission file.

### 4.3 Method

The controller publishes forward velocity commands at 10 Hz. The attacker publishes turn commands at 50 Hz. Both publish into the command path through `/cmd_vel_in`, and the host relay forwards the resulting stream to `/cmd_vel`.

The experiment can be launched with:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

The tmux demo shows three roles:

- `ROBOT_GAZEBO_WSL_HOST`
- `CONTROLLER_DOCKER`
- `ATTACKER_DOCKER`

### 4.4 Expected and Observed Result

The expected normal behavior is straight forward movement. When the attacker starts, the robot trajectory should deviate because turn commands are injected at a higher frequency.

This experiment has been verified in the migrated WSL + Docker environment. The successful relay logs show that container-generated commands are reaching the Gazebo control path.

### 4.5 Security Finding

Default ROS2 communication does not prevent another discovered node from publishing to a critical command topic. Without authentication and authorization, the robot command path is open to unauthorized publishers.

## 5. Experiment 2: SROS2 `/cmd_vel` Access-Control Defense

### 5.1 Purpose

This experiment evaluates whether SROS2 can prevent unauthorized command injection while still allowing legitimate control.

### 5.2 Threat Model

The attacker may still run in the same ROS2 domain, but the system uses SROS2 with enforced security. The controller has permission to publish command input. The attacker does not have permission to publish command topics.

### 5.3 Policy Design

The migrated policy file is:

```text
config/sros2_wsl_docker_policy.xml
```

The policy defines three enclaves:

| Enclave | Role | Permission |
| --- | --- | --- |
| `/controller` | legitimate controller | can publish `/cmd_vel_in` |
| `/gazebo` | Gazebo, bridge, relay | can subscribe `/cmd_vel_in` and publish `/cmd_vel` |
| `/attacker` | unauthorized attacker | cannot publish `/cmd_vel_in` or `/cmd_vel` |

The policy can be validated with:

```bash
./experiments/02_sros2_cmd_vel_defense/validate_policy.sh
```

### 5.4 Jazzy SROS2 Issue and Fix

A key migration issue was the runtime enclave selection. The initial configuration used the wrong assumption about `ROS_SECURITY_ENCLAVE`. In this Jazzy/FastDDS setup, the runtime enclave must be selected using:

```text
ROS_SECURITY_ENCLAVE_OVERRIDE
```

or:

```bash
--ros-args --enclave /controller
```

Without this, the runtime searched the keystore root instead of the specific enclave directory and produced:

```text
couldn't find all security files
```

After switching to `ROS_SECURITY_ENCLAVE_OVERRIDE` and explicit enclave arguments, the security directory resolution was corrected.

### 5.5 Run Procedure

Generate or refresh SROS2 artifacts:

```bash
./scripts/wsl_docker/init_sros2_docker.sh
```

Launch the secure demo:

```bash
./experiments/02_sros2_cmd_vel_defense/run_demo.sh
```

Collect evidence:

```bash
./experiments/02_sros2_cmd_vel_defense/collect_evidence.sh
```

### 5.6 Result Status

The SROS2 experiment has been migrated and policy validation passes. The remaining task is final runtime validation in the real WSL terminal:

- Verify that secure controller traffic still moves the robot.
- Verify that the attacker cannot affect `/cmd_vel_in`.

### 5.7 Security Finding

SROS2 changes the ROS2 graph from an open publication model to an authenticated and permission-controlled model. The important defense is not only encryption, but also authorization: only the controller enclave should be allowed to publish command input.

## 6. Experiment 3: Network MITM / ARP Spoofing

### 6.1 Purpose

This experiment studies a stronger network attacker who attempts to position itself between the controller and robot.

### 6.2 Difference from Experiment 1

Experiment 1 is application-layer command injection:

```text
attacker publishes ROS2 messages
```

Experiment 3 is network-path manipulation:

```text
attacker attempts to become the path between controller and robot
```

This distinction matters because ARP spoofing requires separate layer-2 identities. Docker `network_mode: host` is not appropriate for that attack model.

### 6.3 Migrated Docker Bridge Lab

A separate Docker bridge lab was added:

```text
deployment/wsl_docker/docker-compose.mitm.yml
```

It creates three endpoints:

| Container | Role | IP |
| --- | --- | --- |
| `tb3-mitm-controller` | legitimate endpoint | `172.28.0.10` |
| `tb3-mitm-robot` | target endpoint | `172.28.0.20` |
| `tb3-mitm-attacker` | MITM endpoint | `172.28.0.50` |

The tmux demo is:

```bash
./experiments/03_network_mitm/run_demo.sh
```

### 6.4 ARP Helper

A constrained ARP helper was added:

```text
experiments/03_network_mitm/arp_poison_lab.py
```

It is intentionally limited:

- It defaults to dry-run mode.
- It requires `--execute` to send ARP replies.
- It defaults to the isolated `172.28.0.0/24` Docker lab subnet.
- It supports `--restore`.

Dry-run:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --duration 6
```

Execute in the isolated lab:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20
```

### 6.5 Result Status

The Docker bridge topology, tmux demo, evidence collection, and constrained ARP helper have been migrated. The next step is to connect this network-layer setup to ROS2 command traffic measurement, such as measuring topic visibility or command delivery while ARP state changes.

### 6.6 Security Finding

Network-layer MITM is a different class of risk than publisher injection. SROS2 can protect confidentiality and integrity of DDS messages, but it does not prevent traffic dropping or availability attacks. Network security and ROS2 security should be treated as complementary defenses.

## 7. Experiment 4: SLAM Security Future Work

The final planned direction is SLAM and perception-data security. Command topics such as `/cmd_vel` control robot motion, but sensor and localization topics control the robot's understanding of the world.

Potential targets include:

- `/scan`
- `/odom`
- `/tf`
- `/map`
- camera topics used by ORB-SLAM3

Future work should test whether manipulated sensor or transform data can cause localization drift, mapping errors, or navigation failures.

## 8. Contributions

This project contributes:

1. A maintainable WSL + Docker architecture for TurtleBot3 ROS2 security experiments.
2. A verified open ROS2 command-injection experiment.
3. A migrated SROS2 access-control experiment with policy validation and secure startup scripts.
4. A Docker bridge MITM lab with isolated endpoint identities.
5. A constrained ARP helper for controlled MITM testing.
6. A presentation/report-oriented experiment structure under `experiments/`.
7. tmux-based demos that show robot, controller, and attacker roles in separate panes.

## 9. Limitations

The project has several limitations:

- Gazebo is not containerized in the working setup; it runs on the WSL host for reliable GUI support.
- The SROS2 defense experiment still requires final runtime validation in the user's WSL terminal.
- The Docker bridge MITM lab currently validates topology and ARP manipulation, but still needs ROS2 command-traffic measurement integrated into the MITM phase.
- The SLAM security experiment is documented as future work and has not yet been fully implemented.

## 10. Conclusion

The project successfully migrated a ROS2 TurtleBot3 security workflow from a difficult multi-VM setup to a more maintainable WSL + Docker environment. The migrated system preserves the key security roles while keeping Gazebo visible on the host. The open ROS2 injection experiment demonstrates the risk of unauthenticated command topics. The SROS2 defense work shows how enclave-based permissions can restrict command publishers. The network MITM work establishes an isolated bridge topology for layer-2 security experiments.

Overall, the project demonstrates that ROS2 robot security must be evaluated across multiple layers: application-level topic permissions, middleware authentication, network-path control, and eventually perception-data integrity.
