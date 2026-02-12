# ARP-Based MITM Experiment (Template)

> **Purpose**: Document the high-level workflow for a three-VM, isolated-network experiment where an attacker VM positions itself between the controller VM and the TurtleBot/Gazebo VM by abusing ARP. All packet-crafting logic is deliberately omitted; fill in the TODO sections of the accompanying script template when you conduct the experiment in your controlled lab.

## Roles

| VM | Role | Suggested IP | Notes |
|----|------|--------------|-------|
| VM-A | Controller / Operator | 192.168.56.10 | Publishes legitimate `/cmd_vel` commands |
| VM-B | Attacker / MITM | 192.168.56.50 | Runs ARP MITM helper script | 
| VM-C | Robot / Gazebo | 192.168.56.20 | Receives commands |

Adjust IPs as needed but keep them within the same isolated subnet.

## Prerequisites

1. All VMs share an isolated virtual switch (Bridged/Host-only) and do **not** touch production networks.
2. Python 3 + Scapy installed on the attacker VM: `pip install scapy`.
3. Enable IP forwarding on the attacker VM (temporary):
   ```bash
   sudo sysctl -w net.ipv4.ip_forward=1
   ```
4. (Optional) Configure `iptables`/`nftables` on the attacker VM to forward traffic once the ARP spoof succeeds.

Document each prerequisite in your lab notebook before proceeding.

## Experiment Flow

1. **Baseline capture**
   - From VM-A run `arp -n` and save the output (`controller_arp_before.txt`).
   - Start `tcpdump -nn -e arp` on VM-C to record normal ARP exchanges.

2. **Launch template script**
   - Option A (router forwarding first): `attack_experiments/arp_mitm/scripts/router_forward_template.py`
   - Option B (direct ARP poisoning): `attack_experiments/arp_mitm/scripts/arp_poison_template.py`
   - Both scripts only contain scaffolding. Fill in the `TODO` sections with your own packet-crafting and forwarding logic that is appropriate for your approved research scenario.
   - Recommended workflow:
     ```bash
     cd attack_experiments/arp_mitm/scripts
     python3 arp_poison_template.py \
       --iface eth0 \
       --target-ip 192.168.56.20 \
       --target-mac <fill after discovery> \
       --controller-ip 192.168.56.10 \
       --controller-mac <fill after discovery>
     ```

3. **Observation & logging**
   - Use the script's logging hooks (fill them in) to write JSON/CSV entries describing every ARP request/reply you craft.
   - Keep Wireshark/tcpdump captures running on VM-A and VM-C for correlation.

4. **Validation**
   - Confirm VM-A's ARP cache now lists the attacker MAC for the robot IP.
   - Ensure VM-C sees the attacker MAC for the controller IP.
   - Verify `ros2 topic echo /cmd_vel` still works through the attacker VM (traffic should flow through your forwarding rules).

5. **Cleanup**
   - Disable IP forwarding: `sudo sysctl -w net.ipv4.ip_forward=0`.
   - Flush ARP caches on all VMs (`ip neigh flush all`).
   - Delete temporary firewall rules/captures.

## Deliverables to Record

- Terminal logs (before/after ARP tables, tcpdump summaries).
- The filled-in sections of `arp_poison_template.py` / `router_forward_template.py` (do **not** commit sensitive details; keep private if required by policy).
- Narrative describing the timeline (baseline → spoof attempt → validation → cleanup) for inclusion in your thesis/report.

> 更多中文解析与操作提示，请参见 `README_CN.md`。

## Safety Checklist

- [ ] Performed only inside the three-VM sandbox.
- [ ] No production or campus network interfaces connected.
- [ ] All VMs reverted to clean snapshots after the experiment.
- [ ] Attack logic stored securely and shared only with authorized reviewers.

Keep this document under version control so advisors can audit the preparation steps even if the actual exploit code stays local.
