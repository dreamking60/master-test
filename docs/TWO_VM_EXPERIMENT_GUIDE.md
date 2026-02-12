# 两台虚拟机实验指南

两台虚拟机分别扮演：**目标机（机器人）** 和 **攻击机**。按下面步骤做即可。

---

## 角色说明

| 虚拟机 | 角色 | 作用 |
|--------|------|------|
| **VM1** | 目标机（Target） | 跑 Gazebo + TurtleBot3，被攻击的对象 |
| **VM2** | 攻击机（Attacker） | 跑攻击脚本，向目标机的 `/cmd_vel` 注入指令 |

两台机器要在**同一网络**（能互相 ping 通），且使用**相同的 ROS_DOMAIN_ID**（一般是 0）。

---

## 一、VM1（目标机）要做什么

### 1. 启动 Gazebo + 机器人

```bash
cd /home/stevenchen/master-test/attack_experiments/network_attack/scripts
./setup_target_machine.sh
```

脚本会：
- 自动获取本机 IP
- 询问 ROS_DOMAIN_ID（直接回车即用默认 0）
- **自动启动 Gazebo**（空世界 + TurtleBot3）

### 2. 记下这两项（后面攻击机要填）

终端里会显示类似：

```
Target machine information:
  IP Address: 192.168.x.x    ← 抄下来，给 VM2 用
  ROS_DOMAIN_ID: 0           ← 抄下来，给 VM2 用
```

### 3. 保持 Gazebo 一直开着

- 不要关这个终端
- Gazebo 窗口里能看到机器人在空地上即可

---

## 二、VM2（攻击机）要做什么

### 1. 先配置：指向目标机

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./setup_attacker_machine.sh
```

按提示输入：
- **Target machine IP address**：填 VM1 的 IP（上面记下的）
- **ROS_DOMAIN_ID**：填 VM1 的（一般是 0）

### 2. 再发起攻击

**方式 A：交互式（推荐）**

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./launch_attack_interactive.sh
```

在菜单里选攻击类型（例如 1=左转、2=覆盖前进等）。

**方式 B：命令行直接打**

```bash
cd /path/to/master-test/attack_experiments/network_attack/scripts
./quick_attack.sh turn_left 50 15
# 参数：攻击类型 频率(Hz) 持续时间(秒)
```

### 3. 看效果

- 看 **VM1 的 Gazebo 窗口**：机器人应该按攻击类型运动（例如被强制左转）
- 攻击会按你设的时长自动结束

---

## 三、流程速查

```
VM1（目标机）                          VM2（攻击机）
─────────────────────────────────────────────────────────────
1. 运行 setup_target_machine.sh
2. 记下 IP 和 ROS_DOMAIN_ID    →    3. 运行 setup_attacker_machine.sh
4. Gazebo 保持运行                   5. 输入 VM1 的 IP 和 Domain ID
                                    6. 运行 launch_attack_interactive.sh
                                       或 quick_attack.sh
                                    7. 在 VM1 的 Gazebo 里观察机器人
```

---

## 四、检查是否连通（在 VM2 上）

```bash
# 看能不能发现目标机的节点
ros2 node list

# 看有没有 /cmd_vel
ros2 topic list | grep cmd_vel

# 看 /cmd_vel 实时消息
ros2 topic echo /cmd_vel
```

若能看到 `/cmd_vel` 和节点，说明两台机已连通，可以正常做攻击实验。

---

## 五、常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| VM2 找不到 `/cmd_vel` | Gazebo 没在 VM1 上跑起来 | 在 VM1 先跑 `setup_target_machine.sh`，等 Gazebo 完全起来 |
| VM2 找不到 `/cmd_vel` | IP 或 Domain ID 不对 | 在 VM2 重新跑 `setup_attacker_machine.sh`，填对 VM1 的 IP 和 ROS_DOMAIN_ID |
| VM2 找不到 `/cmd_vel` | 网络不通 | 在 VM2 上 `ping <VM1的IP>`，确保同一网段、无防火墙拦 DDS |
| 攻击没效果 | 频率太低 | 用 50 Hz，例如 `./quick_attack.sh turn_left 50 15` |

---

## 六、实验结束后

- **VM1**：在跑 Gazebo 的终端按 `Ctrl+C` 结束即可。
- 若不再用当前配置，可在目标机脚本目录删除：`target_ip.txt`、`target_domain_id.txt`（若有）。

更细的步骤和故障排查见：`attack_experiments/network_attack/EXPERIMENT_STEPS.md`。
