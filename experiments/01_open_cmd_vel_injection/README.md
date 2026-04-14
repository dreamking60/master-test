# Experiment 01: Open ROS2 `/cmd_vel` Injection

## Purpose

Show that an unsecured ROS2 environment allows an unauthorized publisher to inject velocity commands into the robot control path.

This is the main baseline attack experiment. It should be presented before SROS2, because it establishes the risk that the defense is meant to address.

## Threat Model

- The attacker can join the same ROS2 domain as the controller and Gazebo.
- ROS2 security is disabled.
- The attacker does not need credentials.
- The attacker publishes to the same command path at a higher rate than the legitimate controller.

## Current WSL + Docker Setup

| Role | Location | Node / Script | Topic |
| --- | --- | --- | --- |
| Gazebo robot | WSL host | `scripts/setup/run_wsl_gazebo.sh` | consumes `/cmd_vel` through bridge |
| Relay | WSL host | `scripts/setup/cmd_vel_relay.py` | `/cmd_vel_in -> /cmd_vel` |
| Controller | Docker | `scripts/wsl_docker/controller_forward_pub.py` | publishes `/cmd_vel_in` at 10 Hz |
| Attacker | Docker | `scripts/wsl_docker/attacker_turn_pub.py` | publishes `/cmd_vel_in` at 50 Hz |

## Run Procedure

For presentation/demo use, the recommended command is:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

It opens three tmux panes that represent the Robot/Gazebo WSL host role, Controller Docker role, and Attacker Docker role. Gazebo is not currently containerized in this working setup. The attacker pane waits for Enter by default, so the presenter can first show normal movement and then start the attack.

Terminal 1:

```bash
./scripts/setup/run_wsl_gazebo.sh
```

Terminal 2:

```bash
sudo ./scripts/wsl_docker/start_controller_stack.sh
```

Terminal 3:

```bash
sudo ./scripts/wsl_docker/run_attacker.sh
```

## Expected Result

- Controller only: robot moves forward in a mostly straight line.
- Controller + attacker: robot turns or deviates because attacker publishes turn commands at a higher frequency.
- `/cmd_vel` shows only `cmd_vel_relay` as publisher in the current architecture.
- `/cmd_vel_in` is the topic where controller and attacker compete.

## Key Evidence to Capture

```bash
ros2 topic info /cmd_vel_in -v
ros2 topic info /cmd_vel -v
tail -n 120 test.log | grep -E "cmd_vel_relay|relayed="
tail -n 80 logs/wsl_docker/controller_pub.log
tail -n 80 logs/wsl_docker/attacker_pub.log
```

If trajectory recording is enabled:

```bash
python3 attack_experiments/scripts/record_trajectory.py
python3 attack_experiments/scripts/analyze_trajectory.py
```

## Presentation Points

- Open ROS2 discovery allows an attacker to participate as another publisher.
- The attack does not require modifying the legitimate controller.
- Higher publish frequency can dominate effective robot behavior.
- In this WSL migration, the relay is an engineering workaround, not a security defense.

## Report Paragraph

In the unsecured ROS2 configuration, the controller and attacker are both able to publish velocity commands into the robot command path. The attacker publishes turn commands at a higher rate than the legitimate controller, causing the robot trajectory to deviate from the expected straight-line movement. This demonstrates that default ROS2 communication without authentication or access control is vulnerable to unauthorized command injection.
