# Network Attack Experiment - Step by Step Guide

## 实验准备

你需要：
- **机器1（目标机器）**：当前这台机器，运行 Gazebo 和机器人
- **机器2（攻击机器）**：另一台机器，运行攻击脚本
- 两台机器在同一网络，或可以互相访问

---

## 步骤 1：在目标机器上设置（当前机器）

### 1.1 打开终端，进入实验目录

```bash
cd /home/stevenchen/master-test/attack_experiments/network_attack/scripts
```

### 1.2 运行目标机器设置脚本

```bash
./setup_target_machine.sh
```

### 1.3 按提示操作

脚本会：
1. 自动获取你的机器 IP 地址
2. 询问 ROS_DOMAIN_ID（直接按 Enter 使用默认值 0）
3. 保存配置信息
4. **自动启动 Gazebo**

### 1.4 记录重要信息

脚本会显示：
```
Target machine information:
  IP Address: 192.168.x.x    ← 记下这个 IP！
  ROS_DOMAIN_ID: 0           ← 记下这个！
```

**重要**：把这个 IP 地址和 Domain ID 告诉攻击机器！

### 1.5 保持 Gazebo 运行

- Gazebo 会自动启动
- 看到机器人在空平面上
- **不要关闭这个终端**，保持 Gazebo 运行

---

## 步骤 2：在攻击机器上设置（另一台机器）

### 2.1 在另一台机器上，复制实验文件

你需要把 `network_attack` 文件夹复制到攻击机器，或者直接在攻击机器上：

```bash
# 在攻击机器上，进入项目目录
cd /path/to/master-test/attack_experiments/network_attack/scripts
```

### 2.2 运行攻击机器设置脚本

```bash
./setup_attacker_machine.sh
```

### 2.3 输入目标机器信息

脚本会询问：
1. **Target machine IP address**: 输入步骤 1.4 中记录的 IP 地址
2. **Target machine ROS_DOMAIN_ID**: 输入步骤 1.4 中记录的 Domain ID（通常是 0）

### 2.4 等待配置完成

脚本会：
- 测试网络连接（ping 目标机器）
- 检查是否同一子网
- 如果需要，创建 DDS 路由配置
- 重启 ROS2 daemon
- 测试节点发现

### 2.5 验证连接

如果看到：
```
/cmd_vel topic discovered - ready to attack!
```

说明连接成功！可以继续。

如果看到：
```
/cmd_vel topic not found
```

检查：
- 目标机器的 Gazebo 是否在运行
- IP 地址是否正确
- 网络是否连通

---

## 步骤 3：发起攻击

### 3.1 在攻击机器上，运行攻击脚本

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./launch_attack.sh
```

### 3.2 选择攻击类型

脚本会显示菜单：
```
Select attack type:
  1) Turn left (default)
  2) Override (forward)
  3) Spin
  4) Stealth
  5) Custom
```

输入数字选择（推荐选 1，左转攻击）

### 3.3 观察攻击效果

- 在**目标机器**的 Gazebo 窗口中观察
- 机器人应该开始左转（如果选择了 turn_left）
- 攻击会持续 15 秒（默认）

---

## 快速命令总结

### 目标机器（当前机器）

```bash
# 终端 1：设置并启动
cd /home/stevenchen/master-test/attack_experiments/network_attack/scripts
./setup_target_machine.sh
# 记录显示的 IP 和 Domain ID
# Gazebo 会自动启动，保持运行
```

### 攻击机器（另一台机器）

```bash
# 终端 1：设置攻击机器
cd /path/to/attack_experiments/network_attack/scripts
./setup_attacker_machine.sh
# 输入目标机器的 IP 和 Domain ID

# 终端 2：发起攻击
cd /path/to/attack_experiments/network_attack/scripts
./launch_attack.sh
# 选择攻击类型
```

---

## 手动攻击（可选）

如果你想手动控制攻击参数：

```bash
# 在攻击机器上
cd /path/to/attack_experiments/network_attack/scripts

# 确保环境变量已设置（setup_attacker_machine.sh 已设置）
export ROS_DOMAIN_ID=0  # 与目标机器相同

# 运行攻击
python3 ../../scripts/injection_attack.py \
    --attack-type turn_left \
    --frequency 50 \
    --duration 15 \
    --angular-speed 0.5
```

---

## 验证连接的命令

### 在攻击机器上验证

```bash
# 检查是否能发现目标节点
ros2 node list

# 检查是否能发现 /cmd_vel 话题
ros2 topic list | grep cmd_vel

# 实时查看 /cmd_vel 消息
ros2 topic echo /cmd_vel

# 检查消息频率
ros2 topic hz /cmd_vel
```

---

## 常见问题

### Q: 攻击机器找不到目标机器

**检查清单**：
1. ✅ 目标机器的 Gazebo 是否在运行？
2. ✅ IP 地址是否正确？
3. ✅ ROS_DOMAIN_ID 是否相同？
4. ✅ 网络是否连通？试试 `ping <target_ip>`
5. ✅ 防火墙是否阻止了 DDS 端口？

**解决方法**：
```bash
# 在攻击机器上重启 ROS2 daemon
ros2 daemon stop
ros2 daemon start
sleep 5
ros2 topic list
```

### Q: 攻击没有效果

**检查清单**：
1. ✅ `/cmd_vel` 话题是否存在？
2. ✅ 攻击脚本是否在运行？
3. ✅ 攻击频率是否足够高（建议 50 Hz）？

**解决方法**：
```bash
# 在攻击机器上检查
ros2 topic echo /cmd_vel  # 应该看到消息在发送

# 在目标机器上检查
ros2 topic echo /cmd_vel  # 应该看到相同的消息
```

### Q: 两台机器不在同一子网

脚本会自动检测并创建 DDS 路由配置。确保：
- `dds_attack_config.xml` 文件已创建
- `CYCLONEDDS_URI` 环境变量已设置
- 重启了 ROS2 daemon

---

## 实验完成后的清理

### 停止 Gazebo（目标机器）

在运行 Gazebo 的终端按 `Ctrl+C`

### 清理配置（可选）

```bash
# 删除保存的配置
rm target_ip.txt target_domain_id.txt
rm dds_attack_config.xml  # 如果存在
```

---

## 下一步

实验成功后，你可以：
1. 尝试不同的攻击类型
2. 测试不同的攻击频率
3. 尝试跨子网攻击
4. 测试防御措施（改变 ROS_DOMAIN_ID）

