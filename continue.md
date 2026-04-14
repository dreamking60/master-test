# ROS2 Security Research - Project Progress & Continue Plan

## 1. Project Overview
This project focuses on analyzing and defending against cyber-physical attacks on the TurtleBot3 platform using ROS2. The environment has been migrated from a traditional multi-VM setup to a hybrid **WSL2 + Docker** architecture.

---

## 2. Current Progress (What's Done)

### ✅ Environment Migration & Infrastructure
- Successfully containerized controller and attacker nodes using a shared Docker image.
- **Unified DDS settings**: Changed `ROS_DOMAIN_ID` from `30` to `0` across all scripts and configurations to align with default SROS2 governance policies.
- **Updated Docker Image**: Modified `deployment/wsl_docker/node_image/Dockerfile` to include `ros-jazzy-sros2` and essential message packages.

### ✅ SROS2 Integration (Experiment 2.2.2 - Repaired)
- **Automated Key Generation**: Created `scripts/wsl_docker/init_sros2_docker.sh` to generate SROS2 keystore and enclaves (`/controller`, `/attacker`, `/gazebo`) using a temporary container, bypassing the lack of ROS2 on the WSL host.
- **Security Configuration**: 
    - Updated `docker-compose.yml` with full SROS2 environment variables.
    - Aligned Python node names in `controller_forward_pub.py` and `attacker_turn_pub.py` with their respective SROS2 enclaves (`controller` and `attacker`).
- **File System Adaption**: Fixed directory structure and identified FastDDS security file naming requirements in the Jazzy environment.

### ✅ Injection Attack (Verified)
- Re-verified that the non-secure injection attack works in the new hybrid setup with `ROS_DOMAIN_ID=0`.

---

## 3. Outstanding Issues (What Needs Fixing)

### ⚠️ SROS2 Runtime Validation
- Although the infrastructure is ready, the secure node initialization currently triggers a "couldn't find all security files" error in some RMW implementations. 
- **Next Task**: Debug the exact file structure expected by the Jazzy FastDDS security plugin (potentially missing specific signed permission files or governance path issues).

### ❌ Network Layer MITM (ARP Spoofing)
- **Issue**: Standard ARP spoofing is still impossible in `network_mode: host`.
- **Strategy**: Plan to create a secondary `docker-compose.bridge.yml` that uses a standard Docker bridge network to give containers unique IPs/MACs for this specific experiment.

---

## 4. Next Steps & Action Plan

### Step 1: Finalize SROS2 Secure Run (Immediate)
- [ ] Investigate the `rclpy` security error in Jazzy to ensure the controller can start with `ROS_SECURITY_ENABLE=true`.
- [ ] Verify that the `attacker` node is blocked when it lacks the correct certificate or permission.

### Step 2: Bridge Network for MITM
- [ ] Set up a bridge-mode environment to re-enable ARP Spoofing experiments as per the Mid-Progress Report.
- [ ] Port the `arp_poison.py` logic to work with Docker bridge IPs.

### Step 3: SLAM Security (Long-term)
- [ ] Integrate ORB-SLAM3 into the Docker stack as planned in the future work section of the report.

---
*Last Updated: 2026-04-14*
