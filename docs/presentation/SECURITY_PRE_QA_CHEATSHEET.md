# ROS2 Security Presentation Q&A Cheatsheet

## 1. 最新日志结论

### SROS2 Network MITM recorded run

证据目录：

```text
logs/experiments/03_network_mitm/sros2_recorded_20260420_051959/
```

关键结果：

| Evidence | Value |
| --- | ---: |
| Attack delay/loss | `500ms`, `5% loss` |
| Ping RTT average | `537.7 ms` |
| Ping RTT maximum | `2673.0 ms` |
| SROS2 `/mitm_cmd` last-latency average | `264.9 ms` |
| SROS2 `/mitm_cmd` max latency | `2174.5 ms` |
| SROS2 max command gap | `2531.3 ms` |

ARP 证据：

```text
controller sees 172.28.0.20 at attacker MAC 0a:7e:45:dd:6c:f6
robot sees 172.28.0.10 at attacker MAC 0a:7e:45:dd:6c:f6
attacker ip_forward=1
qdisc netem delay 500ms loss 5%
```

结论：

```text
ARP MITM 成功进入通信路径。SROS2 没有阻止 ARP 欺骗本身，但保护了 DDS payload 的完整性和身份认证。攻击者可以 delay/drop，不能在没有合法密钥和权限的情况下把直行命令合法改成转弯命令。
```

### SROS2 Gazebo MITM visual run

证据文件：

```text
logs/experiments/03_network_mitm/sros2_gazebo_udp_receiver.log
```

关键结果：

| Evidence | Value |
| --- | ---: |
| UDP receiver samples | `175` |
| Average receive rate | `8.37 commands/s` |
| Expected normal rate | `10 commands/s` |
| Maximum ROS command latency | `2996.1 ms` |
| Watchdog stop events | `13` |

结论：

```text
Gazebo 可视化版本中，SROS2 MITM 的正确表现不是“被改成转弯”，而是 command timing 退化：延迟、间隔、watchdog stop。这个实验展示的是 SROS2 下网络层 MITM 对 availability 的影响。
```

## 2. 一句话版回答

### 项目做了什么？

```text
我把 TurtleBot3 ROS2 安全实验从三台 VMware 虚拟机迁移到 WSL + Docker：WSL 负责 Gazebo 可视化，Docker 模拟 controller 和 attacker，然后复现 open command injection、SROS2 access control、DoS availability、ARP MITM 等实验。
```

### 核心结论是什么？

```text
默认 ROS2 缺少 topic-level authentication/authorization，攻击者可以发布控制命令。SROS2 能阻止未授权 command publisher，也能防止 MITM 合法篡改受保护的 DDS payload，但不能单独解决 DoS 或 delay/drop 这类 availability attack。
```

### 最重要的实验链路是什么？

```text
Experiment 01 证明 open ROS2 command topic 可被注入；Experiment 02 证明 SROS2 access control 能阻止未授权 publisher；DoS 和 MITM 实验证明即使 SROS2 防住了篡改和注入，可用性仍然需要网络层防护。
```

## 3. 实验设计问题

### Q1: 为什么要从 VMware 迁移到 WSL + Docker？

答：

```text
原来需要开三台虚拟机，维护成本高，而且 Gazebo GUI、ROS2 环境和攻击机环境耦合在一起。迁移后 WSL host 负责 Gazebo，可视化稳定；Docker 负责 controller、attacker、robot endpoint，角色隔离更清晰，脚本也更容易复现。
```

### Q2: 为什么 Gazebo 放在 WSL host，而不是全部放 Docker？

答：

```text
Gazebo 是 GUI 程序，在 WSL host 运行更稳定，也方便观察机器人轨迹。Docker 主要用来模拟不同机器角色，比如 controller、attacker、MITM robot gateway。这样兼顾可视化和实验隔离。
```

### Q3: 为什么需要 `/cmd_vel_in -> /cmd_vel` relay？

答：

```text
WSL + Docker 下 ROS2 discovery 和 Gazebo bridge 的行为比单机更复杂。relay 把实验控制入口固定在 `/cmd_vel_in`，再由 WSL host 转成 Gazebo 消费的 `/cmd_vel`，这样 controller/attacker 的作用点清晰，Gazebo 侧也更稳定。
```

### Q4: 这个 relay 会不会改变实验结论？

答：

```text
不会改变安全结论。实验关注的是 command channel 是否允许未授权写入、SROS2 是否阻止未授权写入、以及网络延迟/丢包是否影响控制链路。relay 只是工程适配层，攻击和防御仍然发生在 ROS2 command path 上。
```

### Q5: 为什么不用 `network_mode: host` 做 ARP MITM？

答：

```text
ARP 欺骗需要同一个二层网络里的不同 IP/MAC 身份。Docker host network 会共享宿主机网络 namespace，容器没有独立的二层身份，所以不适合验证 ARP spoofing。MITM 实验必须用 Docker bridge 或多 VM LAN。
```

### Q6: Docker bridge 是 NAT，那还能做 ARP MITM 吗？

答：

```text
可以。Docker custom bridge 对外是 NAT，但 bridge 内部的容器在同一个二层广播域，有独立 IP/MAC。ARP spoofing 发生在这个 bridge subnet 内部，不需要 ARP 跨路由器。
```

## 4. Open Injection 问题

### Q7: open ROS2 injection 的本质是什么？

答：

```text
默认 ROS2 graph 里，只要攻击节点能被发现并且知道 topic 类型，就可以创建 publisher 往 command topic 发消息。没有 SROS2 时，系统不会验证这个 publisher 是否有权限控制机器人。
```

### Q8: 为什么 attacker 能影响机器人？

答：

```text
controller 和 attacker 都向同一个 command input 发布消息。Gazebo 最终执行收到的速度命令。如果 attacker 高频发布转向命令，就会与 controller 的正常直行命令竞争，导致轨迹偏移。
```

### Q9: 这是不是漏洞，还是 ROS2 默认设计？

答：

```text
这是未启用安全机制时的默认开放通信模型，不一定是 ROS2 本身的 bug。但在机器人控制场景中，critical topic 如果没有认证和授权，就会形成实际安全风险。
```

### Q10: 为什么 attacker 没显示成 `/cmd_vel` publisher？

答：

```text
当前架构下 attacker 发布的是 `/cmd_vel_in`，WSL relay 是 `/cmd_vel` 的唯一 publisher。这个设计让 Gazebo 只看到 relay，但安全问题仍然在 `/cmd_vel_in` 入口处体现。
```

## 5. SROS2 Defense 问题

### Q11: SROS2 到底保护什么？

答：

```text
SROS2 基于 DDS Security，主要提供身份认证、访问控制、消息完整性和可选加密。具体到我的实验，controller enclave 被允许发布 `/cmd_vel_in`，attacker enclave 没有这个权限，所以创建 command publisher 会失败。
```

### Q12: 实验里如何证明 attacker 被阻止？

答：

```text
FastDDS 报错显示 `/attacker` 的 allow rule 中没有 command topic，publisher 创建失败，例如 topic not found in allow rule。机器人仍然只接收授权 controller 的命令。
```

### Q13: SROS2 是不是让机器人绝对安全？

答：

```text
不是。SROS2 解决的是认证、授权、完整性和保密性。它不能单独解决所有 availability 问题，也不能阻止 ARP spoofing 这种更低层的网络攻击。
```

### Q14: 如果 attacker 拿到了 controller 的 key，会怎样？

答：

```text
那是 credential compromise。攻击者可以冒充合法 controller，这不是正常 MITM 绕过 SROS2，而是密钥泄露后的 endpoint impersonation。报告里要把这两种威胁模型分开。
```

### Q15: 如果把 SROS2 设置成 `Permissive` 会怎样？

答：

```text
`Permissive` 更适合调试，因为安全失败时可能仍允许通信继续。我的防御实验用的是 `Enforce`，这样未授权 publisher 会被明确阻止，结论更严格。
```

## 6. DoS / Availability 问题

### Q16: DoS 是否绕过了 SROS2？

答：

```text
没有。DoS 实验不声称绕过 SROS2。攻击者没有变成合法 publisher，只是通过 DDS/RTPS UDP traffic 造成资源压力和 command-path interruption。
```

### Q17: SROS2 加密认证了，为什么还会受 DoS 影响？

答：

```text
安全协议仍然要接收包、解析头部、调度线程或丢弃无效流量，这些处理都消耗资源。SROS2 保护消息是否合法，但不能让网络和 CPU 资源无限。
```

### Q18: DoS 结果怎么证明？

答：

```text
日志中有三个层面的证据：attacker 发包量和 CPU 占用上升；controller 发布循环出现数秒 stall；relay watchdog 记录一段时间没有收到 `/cmd_vel_in` 并发布 zero `/cmd_vel`。
```

### Q19: 这个是 DDoS 吗？

答：

```text
严格说不是 DDoS，而是单攻击机 DoS。只有多个 attacker 容器或多台机器同时发包，才更接近 distributed DoS。
```

### Q20: DoS 是否让机器人完全失控？

答：

```text
没有。更准确的结论是短时 availability degradation。实验观察到控制链路卡顿和 watchdog stop，但不是永久失效，也不是拿到了控制权限。
```

## 7. MITM 问题

### Q21: MITM 和 open publisher injection 有什么区别？

答：

```text
open injection 是攻击者作为 ROS2 publisher 主动加入 graph 并发布命令。MITM 是攻击者进入 controller 和 robot 的网络路径，目标是观察、延迟、丢弃或修改合法通信。两者威胁模型不同。
```

### Q22: 怎么证明 ARP MITM 成功？

答：

```text
看 ARP 表和 qdisc。攻击期间 controller 把 robot IP 解析成 attacker MAC，robot 把 controller IP 解析成 attacker MAC；attacker 开启 ip_forward，并用 tc netem 加 500ms delay 和 5% loss。这证明流量路径被攻击机接管并转发。
```

### Q23: 为什么要开启 `ip_forward`？

答：

```text
如果 attacker 只做 ARP poisoning 但不转发，结果更像断网或 DoS。开启 ip_forward 后，attacker 成为中继路由，能形成稳定 MITM，再观察 delay/loss/tamper 的影响。
```

### Q24: SROS2 下 MITM 是否可行？

答：

```text
网络层 MITM 可行，因为 ARP spoofing 发生在 SROS2 下面。但 SROS2 保护 DDS payload，所以攻击者不能在没有合法密钥和权限的情况下把受保护命令改成另一个合法命令。
```

### Q25: 那 SROS2 下 MITM 的攻击效果是什么？

答：

```text
主要是 availability attack：delay、drop、rate limit。最新 recorded run 里 ping RTT 平均约 537.7ms，最大约 2673ms；SROS2 command 最大 latency 约 2174.5ms，最大 command gap 约 2531.3ms。
```

### Q26: 为什么 open Gazebo MITM 能让小车转弯，而 SROS2 Gazebo MITM 不应该转弯？

答：

```text
open Gazebo MITM demo 里 robot-gateway 可以改写明文或未受保护的 command，再转发给 Gazebo，所以能直行变转弯。SROS2 demo 里 command 是受 DDS Security 保护的，正常 MITM 不能生成合法篡改后的 payload，所以只展示延迟/丢包导致的卡顿或停顿。
```

### Q27: 如果老师问“为什么你的 SROS2 Gazebo MITM 没有明显转弯”，怎么答？

答：

```text
这正是 SROS2 的预期防御效果。SROS2 不阻止网络层 ARP spoofing，但阻止未授权命令篡改。SROS2 下合理的可视结果是 command timing 退化，而不是合法 command rewrite。
```

### Q28: SROS2 Gazebo MITM 的最新日志说明了什么？

答：

```text
secure UDP receiver 平均接收率从预期 10 commands/s 降到约 8.37 commands/s，最大 ROS command latency 达到 2996.1ms，并触发 13 次 watchdog stop。这说明 Gazebo 可视路径上也出现了 availability degradation。
```

### Q29: 为什么 MITM 实验里用了 `/mitm_cmd` 而不是直接 `/cmd_vel`？

答：

```text
`/mitm_cmd` 是为了把网络 MITM 实验和 Gazebo bridge 的工程细节解耦。controller 到 robot-gateway 的链路用于验证 ARP MITM 对 ROS2/DDS command traffic 的影响；robot-gateway 再把命令转发给 Gazebo。这样可以明确区分网络路径实验和 Gazebo 可视化适配层。
```

### Q30: 这是否说明 SROS2 不需要网络安全？

答：

```text
不是。SROS2 和网络安全是互补的。SROS2 防止 payload 被未授权读取或篡改，但 ARP spoofing、packet drop、delay、flood 仍然需要交换机安全、VLAN、ARP inspection、firewall、QoS、rate limiting 等网络层措施。
```

## 8. 报告和 demo 问题

### Q31: 你会怎么安排 presentation 顺序？

答：

```text
先讲迁移架构，然后讲 open injection 建立风险，再讲 SROS2 defense 证明访问控制有效，然后讲 DoS 和 MITM 说明 SROS2 不是完整 availability 防御，最后讲未来的 SLAM/perception security。
```

### Q32: 哪些实验可以现场 demo，哪些适合放日志？

答：

```text
open injection 和 SROS2 defense 适合现场 demo，因为视觉结果明显。DoS 和 SROS2 MITM 更适合展示 tmux + log evidence，因为它们的效果是 latency、gap、watchdog，而不是必然产生夸张轨迹变化。
```

### Q33: 如果现场 demo 失败怎么办？

答：

```text
我会展示 recorded evidence：summary.md、ARP 表、qdisc、publisher/subscriber latency log、Gazebo UDP receiver log。安全实验不能只依赖现场视觉效果，日志证据更可重复。
```

### Q34: 你的主要 limitation 是什么？

答：

```text
Gazebo 仍在 WSL host 上运行，不是完全容器化；MITM Gazebo 可视化通过 robot-gateway + UDP receiver 适配；SLAM/perception security 还属于 future work。当前结论主要针对 command channel、SROS2 access control、availability 和 network MITM。
```

### Q35: 下一步工作是什么？

答：

```text
下一步可以做 SLAM/perception topic 安全，例如 `/scan`、`/odom`、`/tf`、camera topic 或 map 数据篡改，观察定位漂移、地图错误或导航失败，并测试 SROS2 policy 和 topic validation 是否能降低风险。
```

## 9. 不要说错的话

不要说：

```text
SROS2 可以防止所有 MITM。
DoS 绕过了 SROS2。
SROS2 Gazebo MITM 可以直接把加密命令改成转弯。
Docker NAT 本身就能做 ARP MITM。
我的 DoS 是 DDoS。
```

推荐说：

```text
SROS2 防止未授权发布和 payload 篡改，但不能单独解决 availability。
MITM 在网络层仍然可以成立，但在 SROS2 下只能可靠造成 delay/drop，不能合法 rewrite command。
Docker custom bridge 提供同一二层广播域 and 独立 MAC，是复现 ARP MITM 的关键。
```

## 10. 补充：基础概念与工具细节

### Q36: 实验中那些 ROS 环境变量是什么意思？

| 变量 | 作用 | 简单解释 |
| --- | --- | --- |
| `ROS_DOMAIN_ID=0` | **网络隔离** | 相同 ID 的节点才能通信，不同 ID 逻辑隔离。默认 0。 |
| `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | **中间件选择** | 指定使用 eProsima FastDDS 作为底层通信库。 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` | **传输协议限制** | 强制只用 IPv4 UDP，避免 Docker 环境下共享内存或 IPv6 导致通信失败。 |
| `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` | **发现范围控制** | 限制在当前子网内自动寻找节点，不扫描其他网段。 |
| `ROS_LOCALHOST_ONLY=0` | **跨机开关** | `0` 表示允许通过物理网卡进行跨容器/跨机器通信。 |

### Q37: 安全三要素（SROS2 核心保护目标）的英文？

1.  **机密性 (Confidentiality)**：确保信息不泄密（通过加密 Encryption 实现）。
2.  **完整性 (Integrity)**：确保信息不被篡改（通过数字签名 Signatures 实现）。
3.  **身份验证 (Authentication)**：证明你是谁（通过证书 Certificates 实现）。
*注：DoS 破坏的是 **可用性 (Availability)**。*

### Q38: 如果 SROS2 不能防 DoS，机器人网络靠什么防御？

安全是分层的（Defense in Depth），不能只靠 SROS2：
1.  **网络层隔离**：使用 VLAN 或 VPN（如 WireGuard）将机器人流量与外界物理隔绝。
2.  **OS 级限流**：使用 `iptables` 或 `nftables` 限制 UDP 端口的入包速率。
3.  **流量整形**：使用 Linux `tc` (Traffic Control) 为关键控制流预留带宽。
4.  **DDS 资源限制**：在配置文件中设置 Resource Limits，防止内存被非法包耗尽。
5.  **安全监控**：使用 IDS（入侵检测系统）识别异常的流量洪峰。

### Q39: 你的 MITM 实验具体用了哪些工具？

| 工具 | 实验中的具体应用代码 (Snippet) |
| --- | --- |
| **Scapy** (ARP 欺骗) | `packet = ARP(op=2, pdst=target.ip, hwdst=target.mac, psrc=spoof_ip); send(packet)` |
| **tc + netem** (延迟) | `sudo tc qdisc add dev eth0 root netem delay "${DELAY_MS}ms"` |
| **rclpy** (消息转发) | `self.sub = self.create_subscription(TwistStamped, self.topic, self.cb, 50)` |
| **sysctl** (内核转发) | `sysctl -w net.ipv4.ip_forward=1` |
| **Docker Compose** | 用于隔离 `attacker`、`controller` 和 `robot` 的网络命名空间。 |

