#!/bin/bash
# ROS2 系统状态检查脚本

echo "=========================================="
echo "ROS2 系统状态检查"
echo "=========================================="
echo ""

# 检查 ROS2 环境
echo "=== 1. ROS2 环境 ==="
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 环境未加载"
    echo "   请运行: source /opt/ros/humble/setup.bash (或 jazzy)"
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "⚠️  TURTLEBOT3_MODEL 未设置"
    echo "   建议运行: export TURTLEBOT3_MODEL=burger"
else
    echo "✅ TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
fi
echo ""

# 检查运行中的节点
echo "=== 2. 运行中的节点 ==="
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "⚠️  没有检测到运行中的节点"
    echo "   如果 Gazebo 正在运行，可能需要等待几秒钟"
else
    echo "✅ 检测到以下节点:"
    echo "$NODES"
fi
echo ""

# 检查话题
echo "=== 3. 可用话题 ==="
TOPICS=$(ros2 topic list 2>/dev/null)
if [ -z "$TOPICS" ]; then
    echo "⚠️  没有检测到话题"
    echo "   请确保 Gazebo 仿真正在运行"
else
    echo "✅ 检测到以下话题:"
    echo "$TOPICS" | head -10
    if echo "$TOPICS" | grep -q cmd_vel; then
        echo ""
        echo "✅ /cmd_vel 话题存在（可以控制机器人）"
    fi
fi
echo ""

# 检查进程
echo "=== 4. 相关进程 ==="
if pgrep -f "gazebo" > /dev/null; then
    echo "✅ Gazebo 进程正在运行"
else
    echo "⚠️  Gazebo 进程未检测到"
fi

if pgrep -f "turtlebot3" > /dev/null; then
    echo "✅ TurtleBot3 相关进程正在运行"
else
    echo "⚠️  TurtleBot3 相关进程未检测到"
fi
echo ""

# 检查日志
echo "=== 5. 日志文件 ==="
LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -d "$LOG_DIR" ] && [ "$(ls -A $LOG_DIR 2>/dev/null)" ]; then
    LOG_COUNT=$(find "$LOG_DIR" -name "*.log" 2>/dev/null | wc -l)
    echo "✅ 找到 $LOG_COUNT 个日志文件在 $LOG_DIR"
    LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -n "$LATEST_LOG" ]; then
        echo "   最新日志: $LATEST_LOG"
    fi
else
    echo "⚠️  日志目录为空或不存在"
fi

if [ -f "./test.log" ]; then
    LOG_SIZE=$(du -h "./test.log" | cut -f1)
    echo "✅ test.log 存在 ($LOG_SIZE)"
else
    echo "⚠️  test.log 不存在"
fi
echo ""

# 总结
echo "=========================================="
echo "状态总结"
echo "=========================================="
if [ -n "$ROS_DISTRO" ] && [ -n "$NODES" ] && echo "$TOPICS" | grep -q cmd_vel; then
    echo "✅ 系统看起来正常运行！"
    echo ""
    echo "下一步："
    echo "1. 尝试运行: ros2 run turtlebot3_teleop teleop_keyboard"
    echo "2. 或使用: python3 test_control.py forward 0.2 2.0"
else
    echo "⚠️  系统可能未完全启动"
    echo ""
    echo "建议："
    echo "1. 确保已运行: ./run.sh (启动 Gazebo)"
    echo "2. 等待几秒钟让系统完全启动"
    echo "3. 再次运行此脚本检查状态"
fi
echo "=========================================="

