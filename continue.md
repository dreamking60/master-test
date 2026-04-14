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

### ✅ Injection Attack (Verified)
- Re-verified that the non-secure injection attack works in the new hybrid setup with `ROS_DOMAIN_ID=0`.

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
- [ ] Verify that the `attacker` node is blocked when it lacks the correct certificate or permission.

### Step 2: Bridge Network for MITM
- [x] Set up a bridge-mode environment to re-enable ARP Spoofing experiments as per the Mid-Progress Report.
- [x] Add tmux demo and evidence collection for the bridge lab topology.
- [x] Port the `arp_poison.py` logic to work with Docker bridge IPs in a constrained lab helper.
- [ ] Add ROS2 command traffic measurement while ARP state is changed.

### Step 3: SLAM Security (Long-term)
- [ ] Integrate ORB-SLAM3 into the Docker stack as planned in the future work section of the report.

---
*Last Updated: 2026-04-15*
