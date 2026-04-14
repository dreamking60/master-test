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

The demo starts the isolated Docker bridge lab and shows each endpoint's IP, route table, and ARP table. It does not automatically run ARP poisoning.

Manual commands:

```bash
sudo ./scripts/wsl_docker/mitm_up.sh
sudo ./scripts/wsl_docker/mitm_exec.sh controller "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_exec.sh robot "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_exec.sh attacker "ip -br addr; ip neigh"
sudo ./scripts/wsl_docker/mitm_down.sh
```

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
- ROS2 topic availability before and during MITM.
- Robot trajectory or command log showing whether modification occurred.

## Presentation Points

- ARP spoofing is a network-layer positioning technique, not a ROS2 API attack.
- Docker host networking is useful for ROS2 discovery but unsuitable for L2 ARP experiments.
- This experiment should be presented after the application-layer injection experiment, because it is a deeper network threat model.
- SROS2 should reduce MITM impact because messages are authenticated and encrypted even if the attacker can observe packets.

## Report Paragraph

The MITM experiment studies a stronger attacker who controls the communication path between the controller and robot. Unlike direct ROS2 topic injection, this attack depends on network topology and layer-2 reachability. The current WSL + Docker host-network setup is not appropriate for ARP spoofing because containers do not have independent MAC addresses. A separate Docker bridge or multi-VM topology is required to reproduce this experiment correctly.

## Migration Status

The Docker bridge lab has been added for topology validation and presentation. The next implementation step is to port the ARP/MITM logic into this isolated bridge environment and record before/after ARP state. Active poisoning is intentionally not launched by the tmux demo.
