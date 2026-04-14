# WSL + Docker Hybrid Setup (Gazebo on Host, Controller/Attacker in Containers)

This setup keeps Gazebo visualization on WSL Ubuntu (with WSLg GUI), and runs the control and attack nodes in Docker containers.

## Architecture

- Host WSL Ubuntu:
  - ROS 2 + TurtleBot3 + Gazebo
  - launch simulation and watch robot trajectory
- Docker container `controller`:
  - runs normal operator node
- Docker container `attacker`:
  - runs injection attacker node

All three use the same `ROS_DOMAIN_ID`.

## 1) Host prerequisites (WSL Ubuntu 24.04)

Run the project installer script (it adds ROS2 apt source, installs ROS2 + TurtleBot3 Gazebo + Docker):

```bash
sudo ./scripts/setup/install_wsl_hybrid_deps.sh
```

The script supports:
- Ubuntu 24.04 (`noble`) -> ROS2 Jazzy
- Ubuntu 22.04 (`jammy`) -> ROS2 Humble

It also appends environment defaults to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

After script completes, reload shell/group:

```bash
newgrp docker
source ~/.bashrc
```

## 2) Docker prerequisites (inside WSL)

If you skipped the installer script, install manually:

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2 tmux
```

Check:

```bash
docker --version
docker compose version
```

If `docker compose` is unavailable, install legacy `docker-compose` and rerun:

```bash
sudo apt install -y docker-compose
```

## 3) Start controller/attacker containers

From project root:

```bash
./scripts/wsl_docker/up.sh
```

## One-command tmux demo

For presentation/demo use, launch the three-role view in tmux:

```bash
./scripts/demo/tmux_three_machine_demo.sh open
```

This opens three panes. The panes represent roles/environments, not three Docker containers:

- `ROBOT_GAZEBO_WSL_HOST`: Gazebo + relay + bridge on the WSL host
- `CONTROLLER_DOCKER`: normal controller container
- `ATTACKER_DOCKER`: attacker container

By default the attacker pane waits for Enter, so you can first show the normal robot movement and then trigger the attack during the presentation.

To auto-start the attacker after a delay:

```bash
ATTACK_DELAY=12 ./scripts/demo/tmux_three_machine_demo.sh open
```

For the SROS2 defense demo:

```bash
./scripts/demo/tmux_sros2_defense_demo.sh
```

For the network MITM bridge-lab demo:

```bash
./scripts/demo/tmux_network_mitm_demo.sh
```

## 4) Launch Gazebo on host

In a host shell:

```bash
./scripts/setup/run_wsl_gazebo.sh
```

If you only want to load host env vars into current shell:

```bash
source ./scripts/setup/source_wsl_ros_env.sh
```

## 5) Run normal controller in container

In another shell:

```bash
./scripts/wsl_docker/run_controller.sh
```

Or one command to start containers + controller:

```bash
./scripts/wsl_docker/start_controller_stack.sh
```

## 6) Run attacker in container

After controller is running for a few seconds:

```bash
./scripts/wsl_docker/run_attacker.sh
```

You should observe trajectory deviation in Gazebo when attack commands dominate `/cmd_vel`.

## 7) Secure SROS2 mode

Generate or refresh SROS2 keys and policy-based permissions:

```bash
./scripts/wsl_docker/init_sros2_docker.sh
```

Start Gazebo/relay/bridge with the `/gazebo` enclave:

```bash
./scripts/setup/run_wsl_gazebo_secure.sh
```

Start secure containers and the controller:

```bash
sudo ./scripts/wsl_docker/secure_start_controller_stack.sh
```

Then try the attacker:

```bash
sudo ./scripts/wsl_docker/run_attacker.sh
```

The policy in `config/sros2_wsl_docker_policy.xml` grants `/controller` permission to publish `/cmd_vel_in`, while `/attacker` is not granted command-topic publication permission.

## Useful helper commands

```bash
# topic list from controller container
./scripts/wsl_docker/exec_controller.sh "ros2 topic list"

# watch relay input from attacker container
./scripts/wsl_docker/exec_attacker.sh "ros2 topic echo /cmd_vel_in --once"

# stop containers
./scripts/wsl_docker/down.sh

# collect diagnostics into logs/wsl_docker/diagnostics_*.log
./scripts/wsl_docker/collect_motion_diagnostics.sh
```

### Log files

- Gazebo / bridge launch log: `test.log`
- Controller publisher log: `logs/wsl_docker/controller_pub.log`
- Attacker publisher log: `logs/wsl_docker/attacker_pub.log`
- One-shot diagnostics bundle: `logs/wsl_docker/diagnostics_*.log`

## Troubleshooting

- If nodes cannot discover each other, confirm all shells/containers use same `ROS_DOMAIN_ID`.
- Keep `ROS_AUTOMATIC_DISCOVERY_RANGE` consistent between host and containers (default in this repo: `SUBNET`).
- If discovery is still unstable, keep `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on all participants.
- For strict isolation tests (ARP/L2 MITM), prefer native Linux or VM network instead of Docker bridge abstractions.
