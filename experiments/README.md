# Experiment Index

This directory is the presentation/report-oriented experiment layout. It does not replace the older implementation folders immediately. Instead, each experiment folder explains the purpose, threat model, commands, evidence to collect, and how to discuss the result in a presentation or written report.

## Current Experiment Map

| ID | Experiment | Status | Main Claim |
| --- | --- | --- | --- |
| 01 | Open `/cmd_vel` Injection | Verified in WSL + Docker | An unsecured ROS2 graph allows any discovered publisher to inject robot motion commands. |
| 02 | SROS2 `/cmd_vel` Defense | Migrated, runtime validation next | SROS2 should allow authorized controller traffic while blocking unauthorized command publishers. |
| 03 | Network MITM / ARP Spoofing | Docker bridge lab migrated | Network-path control can support interception/modification, but host-network Docker is not suitable for ARP spoofing. |
| 04 | SLAM Security | Future work | Mapping/localization data integrity should be evaluated after the control-channel experiments are stable. |

## Recommended Presentation Storyline

1. Start with the system architecture: TurtleBot3, Gazebo, ROS2 DDS, controller, attacker, and command topic.
2. Show why the environment had to be migrated: VMware multi-VM setup was expensive to maintain; WSL + Docker isolates roles while keeping Gazebo visible.
3. Demonstrate the baseline vulnerability: open ROS2 topic injection.
4. Demonstrate the defense direction: SROS2 authentication and topic permissions.
5. Explain why network-layer MITM is separate: ARP requires a real L2 topology, which `network_mode: host` does not provide.
6. Close with next work: bridge-mode MITM and SLAM data integrity.

## Evidence Collection Checklist

For each experiment, collect these artifacts:

- Screenshot or short video of Gazebo trajectory.
- `ros2 topic info -v` output for the key topic.
- Controller and attacker logs.
- `/odom` trajectory CSV when possible.
- One short result table: baseline, attack, defense.
- One architecture diagram showing node placement and topic flow.

## Current Stable WSL + Docker Architecture

The working migration path is:

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

The relay exists because direct container-to-Gazebo DDS data delivery was unstable in the WSL + Docker environment even when endpoints were discoverable.
