# Experiment 02: SROS2 `/cmd_vel` Access Control Defense

## Purpose

Show that SROS2 can protect the robot command path by authenticating nodes and enforcing topic-level permissions.

The target result is a direct contrast with Experiment 01:

- Non-secure mode: attacker can inject motion commands.
- Secure mode: authorized controller can control the robot, but attacker cannot publish effective command messages.

## Threat Model

- The attacker can still run inside the same ROS2 domain.
- The attacker may have a certificate for its own enclave.
- The attacker should not have permission to publish command topics.
- SROS2 is enabled with `Enforce` strategy.

## Current Policy Design

Policy file:

```text
config/sros2_wsl_docker_policy.xml
```

Enclaves:

| Enclave | Intended Role | Important Permission |
| --- | --- | --- |
| `/controller` | legitimate controller | can publish `/cmd_vel_in` |
| `/gazebo` | Gazebo, bridge, relay | can subscribe `/cmd_vel_in` and publish `/cmd_vel` |
| `/attacker` | unauthorized attacker | cannot publish `/cmd_vel_in` or `/cmd_vel` |

## Setup

Generate or refresh keys and signed permissions:

```bash
./scripts/wsl_docker/init_sros2_docker.sh
```

The script now prefers host `ros2 security` when available and falls back to Docker if needed.

Validate that the generated policy has the intended command-topic permissions:

```bash
./experiments/02_sros2_cmd_vel_defense/validate_policy.sh
```

## Secure Run Procedure

For presentation/demo use:

```bash
./experiments/02_sros2_cmd_vel_defense/run_demo.sh
```

It opens three tmux panes that represent the secure Gazebo/robot WSL host role, secure controller Docker role, and attacker Docker role. Gazebo is not currently containerized in this working setup.

Equivalent direct demo script:

```bash
./scripts/demo/tmux_sros2_defense_demo.sh
```

Terminal 1:

```bash
./scripts/setup/run_wsl_gazebo_secure.sh
```

Terminal 2:

```bash
sudo ./scripts/wsl_docker/secure_start_controller_stack.sh
```

Terminal 3:

```bash
sudo ./scripts/wsl_docker/run_attacker.sh
```

## Expected Result

Desired final behavior:

- Secure Gazebo/relay starts with enclave `/gazebo`.
- Secure controller starts with enclave `/controller`.
- Controller commands are relayed and the robot moves.
- Attacker process may start, but its `/cmd_vel_in` publisher should not be authorized to communicate with the secure relay.
- Robot should continue following legitimate controller behavior instead of turning under attacker control.

## Current Debug Status

The policy and permissions were regenerated successfully. The generated `/attacker` permissions do not include `rt/cmd_vel_in`, while `/controller` permissions include `rt/cmd_vel_in`.

Remaining issue observed during local secure node initialization:

```text
Docker/Gazebo full secure runtime validation still needs to be run outside the Codex sandbox.
```

Important Jazzy finding:

- `ROS_SECURITY_ENCLAVE` is not the variable that selects the runtime enclave in this setup.
- Use `ROS_SECURITY_ENCLAVE_OVERRIDE=/controller` or pass `--ros-args --enclave /controller`.
- Without the override, rcl looked at the keystore root (`.../keys/enclaves`) instead of the specific enclave (`.../keys/enclaves/controller`) and produced `couldn't find all security files`.
- With `ROS_SECURITY_ENCLAVE_OVERRIDE=/controller`, the security directory resolves correctly. Local validation inside Codex then fails only because the sandbox blocks UDP socket creation, not because security files are missing.

## Key Evidence to Capture

```bash
./experiments/02_sros2_cmd_vel_defense/collect_evidence.sh
```

Manual checks:

```bash
find deployment/wsl_docker/keys/enclaves -maxdepth 2 -type f -o -type l
sed -n '1,180p' deployment/wsl_docker/keys/enclaves/controller/permissions.xml
sed -n '1,180p' deployment/wsl_docker/keys/enclaves/attacker/permissions.xml
ros2 topic info /cmd_vel_in -v
ros2 topic info /cmd_vel -v
```

Evidence logs are written under:

```text
logs/experiments/02_sros2_cmd_vel_defense/
```

## Presentation Points

- SROS2 provides authentication, encryption, integrity, and access control.
- The important security idea is not just "turning on encryption"; it is controlling which enclave can publish sensitive topics.
- A good defense result should show legitimate operation still works while unauthorized command injection fails.
- If the secure run is still being debugged, present it honestly as "policy implemented, runtime validation in progress" and show the generated permissions as evidence of design.

## Report Paragraph

The SROS2 defense experiment applies certificate-based authentication and topic-level permissions to the same robot command path used in the injection experiment. The controller enclave is granted permission to publish command input, while the attacker enclave is intentionally not granted command publication permissions. This design converts the open command channel into an authenticated and permission-controlled channel. The remaining engineering task is validating the Jazzy/FastDDS runtime file layout so the secure nodes can start consistently in the WSL + Docker environment.

## Legacy Migration Note

The old `attack_experiments/sros2_test/` scripts are kept as historical reference. They use a hand-built CA/certificate workflow and older assumptions about keystore layout. The migrated WSL + Docker experiment uses the Jazzy-compatible SROS2 keystore format generated by `ros2 security generate_artifacts` and the policy file in `config/sros2_wsl_docker_policy.xml`.
