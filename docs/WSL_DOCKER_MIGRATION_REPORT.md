# WSL + Docker 迁移排障报告

**项目**: TurtleBot3 ROS2 安全实验迁移  
**环境**: WSL2 Ubuntu 24.04 + ROS2 Jazzy + Gazebo + Docker  
**报告日期**: 2026-03-20

## 1. 目标
将原先 VMware Fusion 三虚拟机实验，迁移为以下混合架构：
- WSL 主机：Gazebo 仿真（可视化）
- Docker 容器：controller（正常控制）+ attacker（攻击节点）

目标是保留“控制机与攻击机容器化”路线，并让小车在 Gazebo 中稳定运动。

## 2. 现象与问题
迁移初期出现典型问题：
- controller/attacker 日志显示持续发布 `/cmd_vel`
- `ros_gz_bridge` 日志显示桥接已创建
- 但 Gazebo 中小车不运动

## 3. 根因分析（最终结论）
根因不是单一脚本 bug，而是**WSL + Docker + DDS 发现/传输链路组合问题**，具体体现在：
1. 主机与容器的 ROS 发现范围/旧变量不一致（`ROS_LOCALHOST_ONLY` 与 discovery range 混用）。
2. 容器直发 `/cmd_vel` 到 Gazebo bridge 的数据面在该环境下不稳定。
3. 迁移后控制链路缺少“主机侧可观测中继点”，排障效率低。

## 4. 修复方案
### 4.1 架构级修复：增加主机侧中继
引入主机 relay 节点：
- 容器发布：`/cmd_vel_in`
- 主机 relay：`/cmd_vel_in -> /cmd_vel`
- Gazebo bridge 继续订阅 `/cmd_vel`

文件：
- `scripts/setup/cmd_vel_relay.py`
- `launch/turtlebot3_empty_world_custom_bridge.launch.py`

### 4.2 消息与桥接统一
统一使用 `geometry_msgs/msg/Twist`，桥接到 `gz.msgs.Twist`，并保留多个 GZ topic fallback：
- `cmd_vel`
- `/model/turtlebot3_burger/cmd_vel`
- `/model/burger/cmd_vel`

文件：
- `config/turtlebot3_burger_bridge_custom.yaml`
- `scripts/wsl_docker/controller_forward_pub.py`
- `scripts/wsl_docker/attacker_turn_pub.py`

### 4.3 环境变量统一（主机与容器）
最终稳定配置：
- `ROS_DOMAIN_ID=30`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
- `ROS_LOCALHOST_ONLY=0`

文件：
- `scripts/setup/source_wsl_ros_env.sh`
- `deployment/wsl_docker/docker-compose.yml`
- `scripts/setup/install_wsl_hybrid_deps.sh`

## 5. 成功证据
`test.log` 中出现以下关键日志（说明数据面已打通）：
- `Relay started: /cmd_vel_in -> /cmd_vel`
- `relayed=100`
- `relayed=200`
- `relayed=300`

这表示容器发出的控制指令已被主机 relay 持续接收并转发。

## 6. 当前可用启动流程
1. 启动 Gazebo（含 relay + bridge）：
```bash
./scripts/setup/run_wsl_gazebo.sh
```
2. 启动容器并运行 controller：
```bash
./scripts/wsl_docker/start_controller_stack.sh
```
3. 运行 attacker：
```bash
sudo ./scripts/wsl_docker/run_attacker.sh
```

## 7. 经验总结
1. 在 WSL + Docker ROS2 混合场景中，看到 topic endpoint 不等于数据一定可达。
2. 要优先构建“可观测数据路径”（本次为主机 relay + 日志计数）。
3. 统一 discovery/transport 配置比单纯改发布脚本更关键。
4. 对 MITM 研究而言，本架构适合注入/控制实验；网络层 MITM 仍建议独立多机网络环境。

## 8. 后续建议
1. 将 relay 计数、bridge 状态、`/odom` 速度监控整合为单个 health-check 脚本。  
2. 保留当前稳定链路作为“实验基线”，再叠加 SROS2/防御模块。  
3. 对网络层 MITM 单独维护一套多机实验拓扑，避免和 WSL 混合环境耦合。
