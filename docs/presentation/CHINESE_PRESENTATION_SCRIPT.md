# 中文 PRE 讲稿提纲：按实验设计来讲

教授希望强调实验设计，所以每个实验都按这个逻辑讲：

```text
目标 -> 假设 -> 实验设计 -> 观测指标 -> 实验结果 -> 支持的结论
```

不要只说“我写了什么脚本”。要说“我为了验证什么安全问题，设计了什么实验”。

## 1. 开场

可以这样说：

```text
我的 master project 关注 ROS2 机器人控制链路的安全性。因为 TurtleBot3 的运动由 ROS2 command topic 控制，所以如果攻击者可以发布、延迟、丢弃或篡改这些消息，软件层攻击就会变成物理运动风险。

因此我的项目不是单纯展示攻击脚本，而是设计了一组可重复实验，分别验证 open ROS2、SROS2 access control、DoS availability 和 network MITM 在 TurtleBot3/Gazebo 环境下的安全影响。
```

## 2. 环境迁移实验设计

目标：

```text
把原来难维护的三虚拟机实验环境迁移到更容易复现的 WSL + Docker 架构，同时保留 robot、controller、attacker 三个安全角色。
```

设计：

```text
WSL host 运行 Gazebo，因为它需要 GUI。
Docker container 运行 controller 和 attacker，因为它们需要隔离角色和可重复环境。
中间用 /cmd_vel_in -> cmd_vel_relay -> /cmd_vel 保证 Docker 到 Gazebo 的控制链路稳定可观测。
```

结论：

```text
这个迁移架构让实验从复杂的多 VM 环境变成可脚本化、可复现的环境，同时仍然能观察 Gazebo 中机器人的物理行为。
```

## 3. Experiment 1: Open ROS2 Command Injection

目标：

```text
验证没有 SROS2 时，未授权 ROS2 节点是否可以影响机器人运动。
```

假设：

```text
如果 ROS2 command topic 没有认证和授权，攻击者只要进入同一个 ROS2 domain，就可以发布控制命令。
```

实验设计：

```text
controller 在 Docker 中以 10Hz 发布直行命令。
attacker 在 Docker 中发布转向命令。
Gazebo 机器人接收 relay 后的 /cmd_vel。
对比 attacker 启动前后的轨迹变化。
```

观测指标：

```text
Gazebo 轨迹是否偏离；/cmd_vel_in 是否出现攻击者 command；relay 是否把 command 传到 Gazebo。
```

结果：

```text
attacker 启动后，机器人轨迹从正常直行变成偏转。
```

结论：

```text
open ROS2 command path 是不安全的，未授权 publisher 可以影响机器人运动。
```

## 4. Experiment 2: SROS2 Access Control Defense

目标：

```text
验证 SROS2 是否能阻止未授权 command publisher，同时允许合法 controller 工作。
```

假设：

```text
如果 policy 只允许 /controller 发布 /cmd_vel_in，而不给 /attacker 这个权限，那么 attacker 应该无法创建合法 command publisher。
```

实验设计：

```text
/controller enclave: 允许 publish /cmd_vel_in。
/gazebo enclave: 允许 subscribe /cmd_vel_in 并 publish /cmd_vel。
/attacker enclave: 不允许 publish command topic。
```

观测指标：

```text
controller 是否仍能控制机器人；attacker 是否报权限错误；机器人轨迹是否不受 attacker 影响。
```

结果：

```text
controller 能正常控制机器人，attacker 创建 publisher 时被 FastDDS/SROS2 拒绝，日志出现 topic not found in allow rule。
```

结论：

```text
SROS2 access control 能有效阻止未授权 command injection。
```

## 5. Experiment 3: SROS2 DoS / Availability

目标：

```text
验证即使 SROS2 阻止了未授权 publisher，系统是否仍可能受到 availability attack 影响。
```

假设：

```text
SROS2 保护身份和权限，但 DDS/RTPS 仍需要处理网络流量。因此高频 UDP flood 可能造成资源压力或 command timing interruption。
```

实验设计：

```text
secure controller 和 secure Gazebo 正常运行。
attacker 不创建合法 publisher，而是扫描并攻击 DDS/RTPS UDP ports，例如 7400 和 7410-7423。
记录 attacker 发包量、controller 发布循环、relay watchdog。
```

观测指标：

```text
packet count、attacker CPU、controller published counter 是否卡顿、relay 是否记录 No /cmd_vel_in。
```

结果：

```text
attacker 发送大量 RTPS-like UDP packets，controller 出现约 3 秒级 stall，relay watchdog 记录 No /cmd_vel_in for 2.93s 并发送 zero /cmd_vel。
```

结论：

```text
DoS 没有绕过 SROS2 权限，但造成了短时 availability degradation。SROS2 不是完整的可用性防御。
```

## 6. Experiment 4: Open Network MITM

目标：

```text
验证攻击者是否能通过 ARP poisoning 成为 controller 和 robot 之间的网络路径中间人。
```

假设：

```text
如果 controller、robot、attacker 在同一个 Docker bridge 里并有独立 IP/MAC，那么 ARP spoofing 可以把通信路径重定向到 attacker。
```

实验设计：

```text
controller: 172.28.0.10
robot: 172.28.0.20
attacker: 172.28.0.50
attacker 开启 ip_forward，执行 ARP poisoning，并用 tc netem 加 delay/loss。
```

观测指标：

```text
ARP table 是否指向 attacker MAC；ping RTT 是否变大；ROS2 command latency/gap 是否增加。
```

结果：

```text
open MITM recorded run 中，ping RTT average 约 547.7ms，ROS2 command max latency 约 2335ms，说明流量确实经过 attacker 并受到影响。
```

结论：

```text
network-path MITM 和 publisher injection 是不同威胁模型，必须用具备独立二层身份的网络拓扑测试。
```

## 7. Experiment 5: SROS2 Network MITM

目标：

```text
验证 SROS2 下，网络层 MITM 能做什么、不能做什么。
```

假设：

```text
SROS2 不会阻止 ARP spoofing，因为 ARP 在 ROS2/DDS Security 之下。但 SROS2 应该阻止 MITM 合法篡改 DDS command payload。攻击者仍然可能 delay/drop。
```

实验设计：

```text
secure controller 使用 /mitm_controller enclave。
secure robot endpoint 使用 /mitm_robot enclave。
attacker 执行 ARP poisoning + 500ms delay + 5% loss，但没有 command permission。
```

观测指标：

```text
ARP table、qdisc、ping RTT、SROS2 command latency、Gazebo UDP receiver watchdog。
```

结果：

```text
SROS2 recorded run 中，ping RTT average 537.7ms，max 2673ms；SROS2 command max latency 2174.5ms，max gap 2531.3ms。
Gazebo visual run 中，max ROS command latency 2996.1ms，并触发 13 次 watchdog stop。
```

结论：

```text
SROS2 不能阻止网络层 MITM 成立，但能阻止合法 payload rewrite。MITM 在 SROS2 下主要造成 delay/drop，也就是 availability degradation。
```

## 8. 总结页讲法

可以这样说：

```text
这些实验共同支持一个分层安全结论：

第一，open ROS2 command topic 会带来直接 command injection 风险。
第二，SROS2 access control 可以阻止未授权 publisher。
第三，SROS2 不能单独解决 availability 问题，DoS 和 network MITM 仍然可以造成 delay/drop。
第四，网络层安全和 DDS/SROS2 安全是互补的，不是互相替代的。
```

## 9. 如果老师追问“你的实验设计贡献是什么？”

回答：

```text
我的贡献不是只写了攻击脚本，而是把不同安全问题拆成可验证的实验：
open injection 测试默认 ROS2 暴露面；
SROS2 defense 测试 authorization；
DoS 测试 availability 边界；
MITM 测试 network-path threat model；
SROS2 MITM 测试 integrity protection 和 availability 的区别。
每个实验都有明确的对照条件、攻击方法、观测指标和安全结论。
```
