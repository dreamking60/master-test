# ARP 实验模板（中文说明）

本目录提供两套仅含模板/骨架的脚本，帮助你在隔离的三台虚拟机实验环境中记录 MITM 实验流程：

1. **router_forward_template.py**（先配置路由转发）
2. **arp_poison_template.py**（ARP 污染/欺骗）

所有真正的报文构造、转发、抓包、ROS2 操作逻辑都需要你自行在 TODO 中补齐。

---

## 1. router_forward_template.py

- **位置**：`attack_experiments/arp_mitm/scripts/router_forward_template.py`
- **适用场景**：攻击机配置为网关/路由器，通过合法路由转发流量，再在转发链路中进行观察/记录。
- **关键函数解析**：
  - `parse_args()`：解析内外网接口、控制端/机器人 IP、日志目录等参数。
  - `ensure_log_dir()`：确保日志目录存在。
  - `log_event()`：将每一步的动作（JSON）写入日志，方便报告引用。
  - `run_cmd()`：封装系统命令调用，`--dry-run` 时只打印命令不执行。
  - `toggle_ip_forward()`：按需开启/关闭 `net.ipv4.ip_forward`。
  - `setup_routing()`：预留 NAT/路由/防火墙配置的 TODO。
  - `monitor_loop()`：监控占位（可挂 scapy.sniff、tcpdump、ROS2 CLI 等）。
  - `teardown_routing()`：清理规则并恢复 IP forwarding。

> 💡 建议先在 `--dry-run` 模式下运行，确认日志与命令输出，再逐步填充实际命令。

---

## 2. arp_poison_template.py

- **位置**：`attack_experiments/arp_mitm/scripts/arp_poison_template.py`
- **适用场景**：攻击机通过伪造 ARP Reply/Request，让控制端与机器人都把攻击机 MAC 当成彼此，从而实现 MITM。
- **关键函数解析**：
  - `parse_args()`：输入接口、控制端/机器人 IP 与 MAC。
  - `craft_arp_reply_template()`：返回占位包对象；你需要在此构造 `Ether()/ARP()` 报文。
  - `send_and_log()`：发送占位包并记录摘要；发送部分留有 TODO。
  - `main()`：展示参数/日志路径，并按顺序调用“伪装控制端”“伪装机器人”两个步骤。

> ⚠️ 记得在实验笔记中记录真实 MAC/IP，以便后期复盘。

---

## 3. 建议的实验记录内容

- 三台虚拟机的 IP、MAC、接口拓扑。
- 执行脚本时的命令、参数、时间戳。
- `arp -n` 或 `ip neigh` 的前/后对比。
- tcpdump/wireshark 抓包摘要。
- ROS2 `/cmd_vel` 的连通性验证（如 `ros2 topic echo` 或轨迹日志）。
- 清理步骤（关闭 IP forwarding、flush ARP、移除 iptables 规则等）。

将这些材料附在论文/实验报告中，可以清楚说明你在隔离环境中完成了哪些验证，同时又不会在仓库存储任何敏感 exploit 代码。
