# ARP 污染验证子实验（最小化流程）

> 目的：在隔离的三台虚拟机环境中，用最少的步骤验证“ARP 污染”是否生效，而不涉及任何进一步的攻击脚本。实验仅验证缓存被篡改的事实，并记录过程。

## 环境准备

| 虚拟机 | 角色 | 示例 IP | 说明 |
|--------|------|---------|------|
| VM-A   | 控制端 | 192.168.56.10 | 执行 `arp -n`、记录缓存 |
| VM-B   | 攻击机 | 192.168.56.50 | 运行 `arp_poison_template.py`（你自行填充 TODO） |
| VM-C   | 机器人 | 192.168.56.20 | 作为被访问目标 |

所有虚拟机必须位于与外界隔离的虚拟交换机内。

## 步骤概览

1. **记录基线**
   - VM-A 执行 `arp -n > before_controller.txt`
   - VM-C 执行 `arp -n > before_robot.txt`
   - 两端同时运行 `tcpdump -nn -e arp -w baseline_controller.pcap` / `...baseline_robot.pcap`

2. **运行模板（攻击机）**
   - `cd attack_experiments/arp_mitm/scripts`
   - `python3 arp_poison_template.py --iface eth0 --controller-ip ... --controller-mac ... --target-ip ... --target-mac ...`
   - 在 `TODO` 中填入真实的 ARP Reply 构造与发送逻辑，仅在此隔离环境执行。

3. **再次采样**
   - VM-A / VM-C 重新执行 `arp -n`，分别保存为 `after_controller.txt`、`after_robot.txt`
   - 抓包转存 `attack_controller.pcap`、`attack_robot.pcap`

4. **对比与判定**
   - 比较 `before_*.txt` 与 `after_*.txt` 是否出现 MAC 地址变化。
   - 在抓包中查找是否存在攻击机 MAC 地址的 ARP Reply。
   - 若控制端与机器人都指向攻击机 MAC，即认为“污染成功”。

5. **清理**
   - 攻击机停止脚本，执行 `ip neigh flush all`。
   - VM-A/VM-C 同样清理缓存，确保网络恢复。
   - 删除实验所用的防火墙/转发配置。

## 需要保存的证据

- `before_*.txt`、`after_*.txt`（ARP 表对比）
- `*.pcap` 抓包文件（可用 Wireshark 进一步截图）
- 攻击机模板脚本的日志 JSON（`logs/arp_mitm/...`）
- 清理命令的终端输出

## 报告建议

- 在实验报告中引用上述文件，并解释“污染成功”的判断依据。
- 如果污染失败，也记录失败原因（如防火墙、静态 ARP、IP 配置错误）以便日后改进。

该子实验聚焦在验证层面，不涉及任何 ROS2 控制指令，适合作为 ARP 研究的基础步骤。
