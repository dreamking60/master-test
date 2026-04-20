# Experiment 03: Network MITM / ARP Spoofing

## Purpose

Evaluate attacks where the attacker is placed in the network path between the controller and robot, instead of simply joining ROS2 as another publisher.

This experiment is separate from the WSL host-network injection setup because ARP spoofing requires distinct layer-2 identities.

## Threat Model

- Controller and robot are on the same layer-2 network or routed through the attacker.
- The attacker can manipulate ARP or act as a gateway/router.
- The attacker attempts to observe, delay, drop, or modify command traffic.

## Why This Is Not the Same as Experiment 01

Experiment 01 is application-level command injection:

```text
attacker publishes ROS2 command messages
```

Experiment 03 is network-path manipulation:

```text
attacker positions itself between controller and robot
```

The security claim is different. Injection proves open ROS2 topics are dangerous. MITM proves network-path control can compromise communication if the transport is not protected.

## Current Migration Issue

The WSL + Docker setup currently uses:

```yaml
network_mode: host
```

This means controller and attacker containers share the host network namespace. They do not have separate Docker bridge IP/MAC identities, so standard ARP spoofing is not meaningful in that topology.

## Migrated Docker Bridge Setup

Separate compose file:

```text
deployment/wsl_docker/docker-compose.bridge.yml
deployment/wsl_docker/docker-compose.mitm.yml
```

Target roles:

| Container | Role | Network Requirement |
| --- | --- | --- |
| controller | legitimate operator | unique IP/MAC |
| robot/relay or simulated robot endpoint | target | unique IP/MAC |
| attacker | MITM host | `NET_ADMIN`, `NET_RAW`, IP forwarding |

Current migrated bridge lab:

| Container | Role | IP |
| --- | --- | --- |
| `tb3-mitm-controller` | legitimate endpoint | `172.28.0.10` |
| `tb3-mitm-robot` | robot/target endpoint | `172.28.0.20` |
| `tb3-mitm-attacker` | MITM endpoint | `172.28.0.50` |

## Run Procedure

For presentation/demo use:

```bash
./experiments/03_network_mitm/run_demo.sh
```

This opens three tmux panes:

- `MITM_CONTROLLER_172.28.0.10`
- `MITM_ROBOT_172.28.0.20`
- `MITM_ATTACKER_172.28.0.50`
- `MITM_MONITOR`

The demo starts the isolated Docker bridge/NAT lab. The controller continuously pings the robot and publishes timestamped ROS2 command messages on `/mitm_cmd`. The robot subscribes to `/mitm_cmd` and reports command latency. When the attacker pane prompts, press Enter to:

1. enable IP forwarding in the attacker container,
2. apply `tc netem` delay/loss on the attacker interface,
3. ARP-poison the controller and robot so traffic is relayed through the attacker,
4. restore ARP entries after the configured duration.

Expected attack evidence:

- controller ping RTT jumps from sub-millisecond/low-millisecond values to approximately the configured delay,
- ICMP TTL changes because packets are routed through the attacker,
- robot packet capture shows ARP poisoning and DDS/RTPS UDP traffic,
- robot ROS2 subscriber logs higher `last_latency_ms`, `avg_latency_ms`, `max_latency_ms`, or `max_gap_ms` during the attack window.

Default attack settings:

```bash
ATTACK_DELAY_MS=250
ATTACK_LOSS_PERCENT=0
ATTACK_DURATION=60
```

Example:

```bash
ATTACK_DELAY_MS=400 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 ./experiments/03_network_mitm/run_demo.sh
```

## Recorded Evidence Run

For report-ready evidence, use the recorded runner:

```bash
ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_SECONDS=45 \
  ./experiments/03_network_mitm/run_recorded_mitm_experiment.sh
```

It automatically runs three phases:

| Phase | Default Duration | Purpose |
| --- | ---: | --- |
| Baseline | `15s` | collect normal ARP, ping RTT, and ROS2 `/mitm_cmd` latency |
| Attack | `45s` | enable forwarding, apply `tc netem`, ARP-poison both endpoints |
| After restore | `15s` | restore ARP/qdisc and verify recovery |

Environment options:

```bash
BASELINE_SECONDS=15
ATTACK_SECONDS=45
AFTER_SECONDS=15
ATTACK_DELAY_MS=500
ATTACK_LOSS_PERCENT=5
```

Output is written to:

```text
logs/experiments/03_network_mitm/recorded_<timestamp>/
```

Important files:

| File | Evidence |
| --- | --- |
| `summary.md` | summarized ping and ROS2 latency statistics |
| `01_before_controller_arp.txt` | controller ARP before attack |
| `02_during_controller_arp.txt` | controller ARP during attack |
| `03_after_controller_arp.txt` | controller ARP after restore |
| `01_before_robot_arp.txt` | robot ARP before attack |
| `02_during_robot_arp.txt` | robot ARP during attack |
| `03_after_robot_arp.txt` | robot ARP after restore |
| `ping_controller_to_robot.log` | end-to-end network RTT evidence |
| `ros2_robot_subscriber.log` | ROS2 command latency/gap evidence |
| `tcpdump_attacker_controller_robot.log` | proof that attacker observed forwarded traffic |
| `02_attack_enable_forward_delay.log` | IP forwarding and qdisc configuration |
| `02_attack_arp_poison.log` | ARP poisoning helper output |

## Gazebo MITM Demonstration

The pure Docker MITM lab proves the network and ROS2 command path impact. For a visual Gazebo demonstration, use:

```bash
ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 \
  ./scripts/demo/tmux_gazebo_mitm_demo.sh
```

Topology:

```text
controller container
  -> ROS2 /mitm_cmd over Docker bridge/NAT
  -> attacker ARP MITM + tc netem delay/loss
  -> robot-gateway container
  -> UDP command forwarding to WSL host gateway 172.28.0.1:15000
  -> WSL UDP receiver publishes /cmd_vel_in
  -> cmd_vel_relay publishes /cmd_vel
  -> Gazebo TurtleBot3
```

This preserves a real ARP MITM path between `controller` and `robot-gateway` while keeping Gazebo visible on the WSL host. The expected visual result is:

- before pressing Enter in the attacker pane: the controller publishes straight commands and the robot moves straight,
- after pressing Enter: the attacker starts ARP MITM, enables delay/loss, and creates the tamper flag; robot-gateway then overrides `angular.z`, so the robot visibly turns instead of following the controller's straight command,
- after the attack duration: ARP state is restored and the tamper flag is removed.

Tamper options:

```bash
MITM_TAMPER_ENABLE=0
MITM_TAMPER_ANGULAR_Z=1.2
```

By default, `MITM_TAMPER_ENABLE=0` means the gateway waits for the attacker pane to create:

```text
logs/experiments/03_network_mitm/mitm_tamper_active
```

This makes the visual demonstration show a clear transition from straight motion to circular/turning motion when the MITM attack begins.

Delay-only visual mode:

```bash
MITM_TAMPER_ENABLE=0 ATTACK_DELAY_MS=800 ATTACK_LOSS_PERCENT=15 \
  ./scripts/demo/tmux_gazebo_mitm_demo.sh
```

Supporting logs:

```text
logs/experiments/03_network_mitm/gazebo_udp_receiver.log
```

The Gazebo demo is intentionally open-mode rather than SROS2 mode. It is meant to demonstrate network-path degradation, while SROS2 experiments separately demonstrate access-control protection.

## SROS2-Protected Gazebo MITM Demonstration

For a Gazebo-visible SROS2 MITM demonstration, use:

```bash
sudo ./scripts/wsl_docker/init_sros2_docker.sh

ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 \
  ./scripts/demo/tmux_sros2_gazebo_mitm_demo.sh
```

Topology:

```text
secure controller container, enclave /mitm_controller
  -> encrypted/authenticated ROS2 /mitm_cmd over Docker bridge/NAT
  -> attacker ARP MITM + tc netem delay/loss
  -> secure robot-gateway container, enclave /mitm_robot
  -> unchanged UDP command forwarding to WSL host gateway 172.28.0.1:15000
  -> secure WSL UDP receiver, enclave /gazebo, publishes /cmd_vel_in
  -> secure cmd_vel_relay, enclave /gazebo, publishes /cmd_vel
  -> Gazebo TurtleBot3
```

Expected result:

- before pressing Enter in the attacker pane, the robot moves straight,
- after pressing Enter, the attacker becomes the network-path MITM and applies delay/loss,
- the robot may stutter, pause, or show less stable command timing,
- the attacker should not be able to transform the protected straight command into a valid protected turn command.

This script intentionally disables command tampering. Under SROS2, a normal network MITM can still delay or drop encrypted DDS traffic, but cannot validly rewrite command contents without compromising the controller or robot-gateway credentials. If a demo shows clean straight-to-turn command modification, that is an open-mode MITM/tamper demo or a credential-compromise scenario, not a normal SROS2 MITM bypass.

Supporting logs:

```text
logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.log
```

## SROS2-Protected MITM Demonstration

SROS2 changes what a MITM attacker can do. It does not stop ARP spoofing itself, but it protects ROS2 command payloads with DDS Security. Therefore:

| Attack Action | SROS2 Result |
| --- | --- |
| ARP poison controller and robot | still possible |
| observe DDS/RTPS packets | still possible, but payload is protected |
| delay/drop/rate-limit packets | still possible |
| modify `/mitm_cmd` into a valid different command | not feasible without valid keys and permissions |
| impersonate controller | not feasible unless controller private keys/permissions are compromised |

Before running this mode, regenerate SROS2 artifacts because the policy includes extra MITM enclaves:

```bash
sudo ./scripts/wsl_docker/init_sros2_docker.sh
```

Interactive demo:

```bash
ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 \
  ./scripts/demo/tmux_sros2_network_mitm_demo.sh
```

Recorded evidence run:

```bash
ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_SECONDS=45 \
  ./experiments/03_network_mitm/run_recorded_sros2_mitm_experiment.sh
```

Expected conclusion:

```text
The attacker can still become a network-path MITM and degrade availability, but cannot transparently tamper with SROS2-protected command content. If the attacker is given the controller's private keys, that is a credential-compromise / endpoint-impersonation scenario rather than a normal MITM bypass.
```

Manual commands:

```bash
sudo ./scripts/wsl_docker/mitm_up.sh
sudo ./scripts/wsl_docker/mitm_exec.sh controller "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_exec.sh robot "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_exec.sh attacker "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_down.sh
```

## ARP Poisoning Lab Helper

The migrated helper is:

```text
experiments/03_network_mitm/arp_poison_lab.py
```

It is intentionally constrained:

- Defaults to dry-run mode.
- Requires `--execute` before it sends ARP replies.
- Defaults to the isolated Docker lab subnet `172.28.0.0/24`.
- Supports `--restore` to send ARP restoration replies when it stops.

Dry-run:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --duration 6
```

Execute only inside the isolated Docker bridge lab:

```bash
sudo ./scripts/wsl_docker/mitm_arp_poison.sh --execute --restore --duration 20
```

Before and after running the execute command, collect ARP evidence:

```bash
sudo ./experiments/03_network_mitm/collect_evidence.sh
```

## NAT vs Bridge Clarification

Docker's custom bridge network is also a NAT network for traffic leaving the subnet. For ARP MITM, the important property is that the three containers share one layer-2 broadcast domain and have distinct IP/MAC identities. A pure layer-3 NAT boundary alone is not enough for ARP spoofing because ARP does not cross routers.

Therefore the migrated topology is:

```text
tb3-mitm-controller 172.28.0.10  \
tb3-mitm-robot      172.28.0.20   > Docker bridge/NAT subnet 172.28.0.0/24
tb3-mitm-attacker   172.28.0.50  /
```

This is the Docker equivalent of the original multi-VM LAN experiment.

## Existing Materials

Useful existing references:

```text
attack_experiments/arp_mitm/
attack_experiments/mitm_attack/
docs/ARP_MITM_ATTACK_IDEA.md
```

## Evidence to Capture

Automated evidence collection:

```bash
sudo ./experiments/03_network_mitm/collect_evidence.sh
```

Evidence logs are written under:

```text
logs/experiments/03_network_mitm/
```

Core evidence:

- IP/MAC table before attack.
- ARP cache before and after attack.
- Packet capture or summarized packet counts.
- ROS2 `/mitm_cmd` subscriber latency before and during MITM.
- Robot trajectory or command log showing whether modification occurred.
- `logs/experiments/03_network_mitm/arp_poison_*.jsonl` from the ARP helper.

## Presentation Points

- ARP spoofing is a network-layer positioning technique, not a ROS2 API attack.
- Docker host networking is useful for ROS2 discovery but unsuitable for L2 ARP experiments.
- This experiment should be presented after the application-layer injection experiment, because it is a deeper network threat model.
- SROS2 should reduce MITM impact because messages are authenticated and encrypted even if the attacker can observe packets.

## Report Paragraph

The MITM experiment studies a stronger attacker who controls the communication path between the controller and robot. Unlike direct ROS2 topic injection, this attack depends on network topology and layer-2 reachability. The current WSL + Docker host-network setup is not appropriate for ARP spoofing because containers do not have independent MAC addresses. A separate Docker bridge or multi-VM topology is required to reproduce this experiment correctly.

## Migration Status

The Docker bridge lab, tmux topology demo, evidence collector, and constrained ARP poisoning helper have been migrated. The next implementation step is to connect this network-positioning experiment back to ROS2 command traffic measurements, for example by running a controller publisher and robot subscriber across the bridge while ARP state is changed.
