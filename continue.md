# ROS2 Security Research - Project Progress & Continue Plan

## 1. Project Overview
This project focuses on analyzing and defending against cyber-physical attacks on the TurtleBot3 platform using ROS2. The environment has been migrated from a traditional multi-VM setup to a hybrid **WSL2 + Docker** architecture.

---

## 2. Current Progress (What's Done)

### ✅ Presentation-Oriented Experiment Organization
- Added a top-level `experiments/` directory that separates the project into report-friendly experiment folders:
    - `experiments/01_open_cmd_vel_injection/`
    - `experiments/02_sros2_cmd_vel_defense/`
    - `experiments/03_network_mitm/`
    - `experiments/04_slam_security/`
- Added `docs/presentation/PRESENTATION_AND_REPORT_GUIDE.md` with slide structure, result table, figures to prepare, and report-writing guidance.
- Added `docs/FINAL_REPORT_DRAFT.md` as a final report draft based on the mid-progress report and migrated experiments.
- Added `docs/presentation/PPT_BUILD_PLAN.md` as a slide-by-slide final presentation plan.
- Added `scripts/demo/tmux_three_machine_demo.sh` to launch a three-pane demo view: WSL host Gazebo/robot, Docker controller, and Docker attacker.
- Added `scripts/demo/tmux_sros2_defense_demo.sh` as the dedicated tmux entry point for Experiment 02.
- Added `scripts/demo/cleanup_all_experiments.sh`; tmux demos now run cleanup by default to avoid cross-experiment interference.
- Existing scripts were not moved yet to avoid breaking the working WSL + Docker pipeline; the new folders currently act as experiment guides and indexes.

### ✅ Environment Migration & Infrastructure
- Successfully containerized controller and attacker nodes using a shared Docker image.
- **Unified DDS settings**: Changed `ROS_DOMAIN_ID` from `30` to `0` across all scripts and configurations to align with default SROS2 governance policies.
- **Updated Docker Image**: Modified `deployment/wsl_docker/node_image/Dockerfile` to include `ros-jazzy-sros2` and essential message packages.

### ✅ SROS2 Integration (Experiment 2.2.2 - Repaired)
- **Automated Key Generation**: Updated `scripts/wsl_docker/init_sros2_docker.sh` to generate SROS2 keystore and enclaves (`/controller`, `/attacker`, `/gazebo`) using host `ros2 security` when available, with Docker fallback.
- **Security Configuration**: 
    - Updated `docker-compose.yml` with full SROS2 environment variables.
    - Aligned Python node names in `controller_forward_pub.py` and `attacker_turn_pub.py` with their respective SROS2 enclaves (`controller` and `attacker`).
    - Replaced the incorrect `ROS_SECURITY_ENCLAVE` assumption with Jazzy-compatible `ROS_SECURITY_ENCLAVE_OVERRIDE` and explicit `--ros-args --enclave ...` in run scripts.
- **Policy Design**:
    - Added `config/sros2_wsl_docker_policy.xml`.
    - `/controller` is allowed to publish `/cmd_vel_in`.
    - `/gazebo` is allowed to subscribe `/cmd_vel_in` and publish `/cmd_vel`.
    - `/attacker` is intentionally not allowed to publish `/cmd_vel_in` or `/cmd_vel`.
- **Secure Startup Scripts**:
    - Added `scripts/setup/run_wsl_gazebo_secure.sh`.
    - Added `scripts/wsl_docker/secure_up.sh`.
    - Added `scripts/wsl_docker/secure_start_controller_stack.sh`.
- **Experiment 02 Migration Helpers**:
    - Added `experiments/02_sros2_cmd_vel_defense/run_demo.sh`.
    - Added `experiments/02_sros2_cmd_vel_defense/validate_policy.sh`.
    - Added `experiments/02_sros2_cmd_vel_defense/collect_evidence.sh`.
- **File System Adaption**: Fixed directory structure and identified FastDDS security file naming requirements in the Jazzy environment.

### ✅ New Experiment Draft: SROS2 UDP DoS / Availability Attack
- This is a separate experiment from the SROS2 access-control defense experiment.
- It reuses the successful SROS2 secure baseline only as a control condition: secure Gazebo/relay with enclave `/gazebo`, secure controller with enclave `/controller`, and the robot moving normally before any attack starts.
- The first bug found in `scripts/demo/tmux_sros2_dos_demo.sh` was that it started non-secure Gazebo with `scripts/setup/run_wsl_gazebo.sh` while starting a secure controller. That mismatch explains why the controller failed before the attacker began.
- The DoS tmux script has been adjusted to start `scripts/setup/run_wsl_gazebo_secure.sh` and `scripts/wsl_docker/secure_start_controller_stack.sh`, matching the known-good SROS2 defense baseline.
- The attacker pane now waits for manual confirmation by default, so the baseline can be observed first.
- New observation: if the controller stops but the robot keeps moving, that does not necessarily mean the DoS attack succeeded or failed. The Gazebo velocity plugin can keep applying the last received non-zero `/cmd_vel` command.
- Mitigation added for cleaner experiments:
    - `scripts/wsl_docker/controller_forward_pub.py` publishes a zero `Twist` before normal shutdown.
    - `scripts/setup/cmd_vel_relay.py` has a 0.5s input watchdog; if `/cmd_vel_in` stops arriving, it publishes a zero `/cmd_vel`.
- Initial DoS result: the robot continued moving normally after the UDP flood started. With the relay watchdog now validated, this is a negative result for the first DoS variant rather than an environment failure.
- Follow-up observation: square-path movement is useful, but raw square trajectories naturally vary slightly even without attack. The DoS result should therefore be judged with logs/metrics and a visible trajectory trace, not by small shape differences alone.
- The DoS generator was revised from fixed-port random UDP flooding to local DDS port discovery plus RTPS-like payloads. The tmux monitor now shows Docker resource usage, DDS UDP sockets, and controller logs during the attack.
- The controller now supports `CONTROLLER_PATTERN=square` in addition to the default forward mode. The DoS tmux demo uses the square path so timing delay, packet loss, or control-loop disruption is easier to see than with continuous straight-line motion.
- Added Gazebo-visible trajectory marking:
    - `scripts/setup/trajectory_trail_spawner.py` subscribes `/odom` and spawns orange ground discs in Gazebo as the robot moves.
    - The first implementation used inline SDF in `gz service --req`, but the markers were not visible and the failure mode was hard to inspect. It was revised to write marker SDF files into `/tmp/tb3_trajectory_trail/` and call Gazebo with `sdf_filename`, while logging successful and failed marker creation.
    - The node is launched from `launch/turtlebot3_empty_world_custom_bridge.launch.py`, so both the open injection experiment and the SROS2/DoS experiments get visible trail markers.
    - The SROS2 policy grants `/gazebo` enclave node `trajectory_trail_spawner` permission to subscribe `/odom`; regenerate SROS2 artifacts after this policy change.
- Added log-driven validation automation:
    - `scripts/experiments/run_log_driven_validation.sh` runs `open-injection`, `sros2-dos`, or `all` without tmux and saves evidence under `logs/experiments/log_driven_validation/<timestamp>/`.
    - `scripts/experiments/summarize_trajectory.py` splits trajectory CSVs into baseline / attack / after phases and reports distance, yaw change, average velocities, sample gaps, and a conservative classification.
    - `scripts/setup/install_codex_sudoers.sh` installs a limited NOPASSWD sudoers file for only the project scripts needed by automation; it does not grant global Docker/root access.
    - `scripts/wsl_docker/docker_status.sh` is the bounded sudoable wrapper used to collect Docker ps/stats evidence.
    - This is intended for evaluating attack impact from logs and trajectory metrics instead of relying only on visual inspection.
- Experimental limitation: random UDP payloads may be dropped during RTPS parsing before SROS2 signature verification. Therefore this should currently be described as a DDS/RTPS UDP flood availability test, not as proven cryptographic-verification exhaustion.

### ✅ Injection Attack (Verified)
- Re-verified that the non-secure injection attack works in the new hybrid setup with `ROS_DOMAIN_ID=0`.

### ✅ Experiment Isolation / Cleanup
- Added a shared cleanup script:
    - `scripts/demo/cleanup_all_experiments.sh`
- The cleanup script stops previous experiment state before a new demo:
    - Kills tmux sessions: `tb3_security_demo`, `tb3_sros2_defense_demo`, `tb3_network_mitm_demo`
    - Stops host-network controller/attacker containers via `scripts/wsl_docker/down.sh`
    - Stops MITM bridge containers via `scripts/wsl_docker/mitm_down.sh`
    - Stops leftover Gazebo / `ros2 launch` / `gz sim` / `cmd_vel_relay.py` processes
    - Resets the ROS2 daemon
- The main tmux demo scripts call cleanup automatically before starting:
    - `scripts/demo/tmux_three_machine_demo.sh open`
    - `scripts/demo/tmux_sros2_defense_demo.sh`
    - `scripts/demo/tmux_network_mitm_demo.sh`
- If an existing environment must be preserved, run demos with:
```bash
SKIP_CLEANUP=1 ./scripts/demo/tmux_three_machine_demo.sh open
```
- Reason: Experiment 01 and Experiment 02 share the WSL Gazebo + Docker controller/attacker path, so stale publishers, stale Gazebo processes, old Docker containers, or ROS2 daemon cache can invalidate experiment conclusions.

---

## 3. Outstanding Issues (What Needs Fixing)

### ⚠️ SROS2 Runtime Validation
- Policy-based permissions now generate successfully, and `/attacker` no longer has `rt/cmd_vel_in` permission.
- The previous `couldn't find all security files` error was traced to enclave resolution. Jazzy resolves correctly when using `ROS_SECURITY_ENCLAVE_OVERRIDE=/controller` or `--ros-args --enclave /controller`.
- Local validation inside the Codex sandbox then reaches FastDDS participant creation but fails at UDP socket creation due to sandbox restrictions. This is not a keystore error.
- **Next Task**: Run the secure Gazebo + secure controller stack in the real WSL terminal and confirm robot movement, then run attacker and confirm it cannot affect `/cmd_vel_in`.

### ❌ Network Layer MITM (ARP Spoofing)
- **Issue**: Standard ARP spoofing is still impossible in `network_mode: host`.
- **Progress**: Added a separate Docker bridge lab in `deployment/wsl_docker/docker-compose.mitm.yml` with independent controller, robot, and attacker containers.
- **Demo/Evidence**:
    - Added `scripts/demo/tmux_network_mitm_demo.sh`.
    - Added `experiments/03_network_mitm/run_demo.sh`.
    - Added `experiments/03_network_mitm/collect_evidence.sh`.
- **ARP Helper**:
    - Added constrained helper `experiments/03_network_mitm/arp_poison_lab.py`.
    - Added wrapper `scripts/wsl_docker/mitm_arp_poison.sh`.
    - Helper defaults to dry-run and requires `--execute`; defaults to the isolated `172.28.0.0/24` lab subnet.
- **Next Task**: Connect the bridge MITM experiment to ROS2 command traffic measurements.

---

## 4. Next Steps & Action Plan

### Step 1: Finalize SROS2 Secure Run (Immediate)
- [x] Add policy-based SROS2 permissions for controller/gazebo/attacker.
- [x] Add secure startup scripts.
- [x] Fix `couldn't find all security files` during secure node startup by using `ROS_SECURITY_ENCLAVE_OVERRIDE` / `--ros-args --enclave`.
- [x] Add Experiment 02 folder-level run, policy validation, and evidence collection helpers.
- [ ] Validate that the controller can start with `ROS_SECURITY_ENABLE=true` in the real WSL terminal.
- [x] Verify that the `attacker` node is blocked when it lacks the correct certificate or permission. Runtime symptom: FastDDS refuses to create the `/cmd_vel_in` data writer for `/attacker`.
- [x] Fix `tmux_sros2_defense_demo.sh` startup race/command-concatenation issue. The root wrapper calls `tmux_three_machine_demo.sh secure`; the shared tmux script now separates the banner command from the pane command with an explicit shell separator, so `read -r -p ...` is no longer accidentally appended to `printf '\n'`. The attacker pane also runs `secure_up.sh` before `run_attacker.sh`, so `service "attacker" is not running` should not occur if Docker itself is healthy.

### Step 1B: SROS2 UDP DoS / Availability Experiment
- [x] Compare the DoS tmux script against the known-good SROS2 defense startup path.
- [x] Fix the baseline mismatch: secure controller must not be paired with non-secure Gazebo/relay.
- [x] Validate relay/controller stop behavior so the robot does not keep executing stale `/cmd_vel` after controller shutdown.
- [x] Add square-path controller mode for DoS observation.
- [x] Add Gazebo-visible `/odom` trajectory dots for DoS and open injection demos.
- [x] Add non-tmux log-driven validation runner and trajectory summary tool.
- [x] Add limited sudoers installer and bounded Docker status helper for non-interactive evidence collection.
- [ ] Re-run `scripts/demo/tmux_sros2_dos_demo.sh` with discovered DDS ports and record whether CPU, message rate, or trajectory changes.
- [ ] Install the project sudoers whitelist once with `sudo ./scripts/setup/install_codex_sudoers.sh`.
- [ ] Run `scripts/experiments/run_log_driven_validation.sh all` in the real WSL terminal after sudo NOPASSWD is configured, then inspect generated evidence directories.
- [ ] If the attack has no measurable impact, refine the traffic generator so it produces more realistic RTPS-shaped traffic rather than pure random UDP payloads.

### Step 2: Bridge Network for MITM
- [x] Set up a bridge-mode environment to re-enable ARP Spoofing experiments as per the Mid-Progress Report.
- [x] Add tmux demo and evidence collection for the bridge lab topology.
- [x] Port the `arp_poison.py` logic to work with Docker bridge IPs in a constrained lab helper.
- [x] Clarify MITM topology: Docker custom bridge provides the required same-L2 ARP domain and NAT for outside traffic. A pure L3 NAT boundary alone cannot support ARP spoofing because ARP does not cross routers.
- [x] Upgrade `tmux_network_mitm_demo.sh` into an active ARP MITM demonstration. Controller continuously pings robot; attacker enables IP forwarding, applies `tc netem` delay/loss, executes constrained ARP poisoning, then restores ARP state; monitor shows ARP/qdisc evidence live.
- [x] Upgrade the network MITM lab from ICMP-only evidence to ROS2 command-path evidence. The controller now publishes timestamped `/mitm_cmd` messages, the robot subscribes and reports command latency/gaps, and the robot pane captures ARP/ICMP/DDS UDP traffic. The expected MITM effect is visible in both ping RTT and ROS2 message latency.
- [x] Add `experiments/03_network_mitm/run_recorded_mitm_experiment.sh` for report-ready evidence collection. It runs baseline/attack/after phases, records ARP tables, ping RTT, attacker tcpdump, robot DDS tcpdump, qdisc/IP forwarding state, ROS2 publisher/subscriber logs, ARP poisoning logs, and emits `summary.md` under `logs/experiments/03_network_mitm/recorded_<timestamp>/`.
- [x] Add Gazebo MITM demo using a robot-gateway architecture: controller publishes ROS2 `/mitm_cmd` inside the Docker MITM network, attacker ARP-poisons controller and robot-gateway and applies delay/loss, robot-gateway forwards commands by UDP to the WSL host, and a host UDP receiver publishes `/cmd_vel_in` for Gazebo. Entry point: `scripts/demo/tmux_gazebo_mitm_demo.sh`.
- [x] Add tamper mode to the Gazebo MITM path. `mitm_robot_udp_forwarder.py` can override forwarded `angular.z` before sending UDP commands to Gazebo, so the visual demo can show the robot turning even though the controller publishes straight commands. Default behavior now waits for the attacker pane to create `logs/experiments/03_network_mitm/mitm_tamper_active`, so the robot starts straight and only turns after the MITM attack is triggered.
- [x] Add SROS2-protected network MITM variant. Policy now includes `/mitm_controller` and `/mitm_robot` enclaves for `/mitm_cmd`; `mitm_secure_up.sh`, `tmux_sros2_network_mitm_demo.sh`, and `run_recorded_sros2_mitm_experiment.sh` demonstrate that ARP MITM can still delay/drop encrypted DDS traffic but cannot transparently modify command content without credential compromise.
- [x] Add SROS2-protected Gazebo MITM visual variant. Entry point: `scripts/demo/tmux_sros2_gazebo_mitm_demo.sh`. WSL Gazebo and the UDP receiver run under `/gazebo`, controller runs under `/mitm_controller`, and robot-gateway runs under `/mitm_robot`. The attacker can ARP-poison and apply delay/loss, but tamper mode is intentionally disabled because SROS2 should prevent valid command rewriting without key/permission compromise.
- [x] Replace fragile multi-line `tmux send-keys` commands in `tmux_sros2_mitm_demo.sh` with a dedicated pane runner script: `scripts/demo/sros2_mitm_pane.sh`. This makes controller startup explicit and prevents command fragments such as `printf ... read -r -p ...` from being pasted/executed incorrectly inside tmux panes.
- [x] Add explicit ROS2 static discovery peers for the bridge MITM demo. WSL host Gazebo uses `ROS_STATIC_PEERS=172.18.0.10;172.18.0.50`; bridge containers use `ROS_STATIC_PEERS=172.18.0.1`. This targets the current failure where the bridge controller publishes `/cmd_vel_in` but the host `cmd_vel_relay` never logs `relayed=...`.
- [x] Add a `zigzag`/`weave` controller pattern for network-delay demos. Square paths are too coarse to show delay clearly; the new pattern keeps forward motion while alternating left/right angular velocity every short period, making command latency or jitter easier to see in Gazebo and in trajectory markers.
- [x] Stop recreating bridge containers when the attacker pane starts. Recreating the compose stack can interrupt the running controller and invalidate the observation.
- [x] Adjust the bridge attacker demo to use a clearer visual setup: the controller drives straight forward continuously, while the attacker attempts to inject turn commands on `/cmd_vel_in`. In secure mode, the expected security result is that the attacker publisher is blocked and the robot keeps moving straight; in an open variant, the same attacker would visibly bend the path.
- [x] Remove `docker compose up` from the attacker pane. The controller pane owns bridge-stack startup; the attacker pane now only waits for `tb3-attacker-bridge` to be running and then executes the turn-injection node. This avoids slow duplicate build/start work delaying the attack phase.
- [x] Add ROS2 command traffic measurement while ARP state is changed. Recorded runners now capture ping RTT, ARP tables, qdisc state, tcpdump, and ROS2 command latency/gap logs for both open and SROS2 MITM variants.

### Step 3: SLAM Security (Long-term)
- [ ] Integrate ORB-SLAM3 into the Docker stack as planned in the future work section of the report.

---
*Last Updated: 2026-04-15*
