#!/bin/bash
# TurtleBot3 Teleop 快速诊断脚本

echo "=========================================="
echo "TurtleBot3 Teleop 诊断工具"
echo "=========================================="
echo ""

echo "=== 1. ROS2 环境检查 ==="
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS_DISTRO 未设置！"
    echo "   请运行: source /opt/ros/humble/setup.bash (或 jazzy)"
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "❌ TURTLEBOT3_MODEL 未设置！"
    echo "   请运行: export TURTLEBOT3_MODEL=burger"
else
    echo "✅ TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
fi
echo ""

echo "=== 2. 包检查 ==="
if ros2 pkg list | grep -q turtlebot3_teleop; then
    echo "✅ turtlebot3_teleop 包已安装"
else
    echo "❌ turtlebot3_teleop 包未找到！"
    echo "   请运行: sudo apt install ros-$ROS_DISTRO-turtlebot3-teleop"
fi
echo ""

echo "=== 3. 话题检查 ==="
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ /cmd_vel 话题存在"
    echo "   话题信息:"
    ros2 topic info /cmd_vel 2>/dev/null | head -3
else
    echo "❌ /cmd_vel 话题不存在！"
    echo "   请确保 Gazebo 仿真正在运行 (Step 10)"
fi
echo ""

echo "=== 4. 节点检查 ==="
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "⚠️  没有检测到运行中的节点"
    echo "   请确保 Gazebo 仿真正在运行 (Step 10)"
else
    echo "✅ 检测到以下节点:"
    echo "$NODES" | head -5
fi
echo ""

echo "=== 5. 测试发布命令 ==="
echo "尝试发布一个测试命令（机器人应该短暂前进）..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ 命令发布成功！如果机器人移动了，说明接口正常。"
else
    echo "❌ 命令发布失败！请检查话题是否存在。"
fi
echo ""

echo "=== 6. 终端模式检查 ==="
if [ -t 0 ]; then
    echo "✅ 终端支持交互式输入"
    TERM_MODE=$(stty -g 2>/dev/null)
    if [ $? -eq 0 ]; then
        echo "   当前终端模式: $TERM_MODE"
    fi
else
    echo "⚠️  当前可能不是交互式终端"
    echo "   如果通过 SSH 连接，建议在虚拟机本地终端运行 teleop_keyboard"
fi
echo ""

echo "=========================================="
echo "诊断完成！"
echo ""
echo "如果所有检查都通过但键盘控制仍不工作，"
echo "可能是终端输入问题。建议："
echo "1. 在虚拟机本地终端（非 SSH）运行 teleop_keyboard"
echo "2. 或使用 ros2 topic pub 命令直接控制（见故障排除指南）"
echo "=========================================="

