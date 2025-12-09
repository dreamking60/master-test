#!/bin/bash
# 测试机器人是否能够移动

echo "=========================================="
echo "机器人移动测试"
echo "=========================================="
echo ""

# 确保 ROS2 环境已加载
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

export TURTLEBOT3_MODEL=burger

echo "步骤 1: 获取初始位置..."
INITIAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)
if [ -z "$INITIAL_POSE" ]; then
    echo "❌ 无法获取初始位置，请确保 Gazebo 正在运行"
    exit 1
fi

INITIAL_X=$(echo "$INITIAL_POSE" | grep "x:" | awk '{print $2}')
INITIAL_Y=$(echo "$INITIAL_POSE" | grep "y:" | awk '{print $2}')

echo "初始位置: x=$INITIAL_X, y=$INITIAL_Y"
echo ""

echo "步骤 2: 发送前进命令（0.2 m/s，持续 3 秒）..."
# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/test_control.py" forward 0.2 3.0 &
CONTROL_PID=$!

# 等待命令执行完成
wait $CONTROL_PID
echo ""

echo "步骤 3: 等待 1 秒让位置稳定..."
sleep 1
echo ""

echo "步骤 4: 获取最终位置..."
FINAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)
if [ -z "$FINAL_POSE" ]; then
    echo "❌ 无法获取最终位置"
    exit 1
fi

FINAL_X=$(echo "$FINAL_POSE" | grep "x:" | awk '{print $2}')
FINAL_Y=$(echo "$FINAL_POSE" | grep "y:" | awk '{print $2}')

echo "最终位置: x=$FINAL_X, y=$FINAL_Y"
echo ""

echo "步骤 5: 计算移动距离..."
# 使用 Python 进行精确计算
DELTA_X=$(python3 -c "print($FINAL_X - $INITIAL_X)")
DELTA_Y=$(python3 -c "print($FINAL_Y - $INITIAL_Y)")
DISTANCE=$(python3 -c "import math; print(math.sqrt($DELTA_X**2 + $DELTA_Y**2))")

echo "位置变化: Δx=$DELTA_X 米, Δy=$DELTA_Y 米"
echo "移动距离: $DISTANCE 米"
echo ""

echo "=========================================="
echo "测试结果"
echo "=========================================="

# 判断是否移动（阈值 0.01 米）
THRESHOLD=0.01
MOVED=$(python3 -c "print(1 if $DISTANCE > $THRESHOLD else 0)")
if [ "$MOVED" = "1" ]; then
    echo "✅ 成功！机器人移动了 $DISTANCE 米"
    echo "✅ 控制接口 /cmd_vel 工作正常"
    echo ""
    echo "下一步：可以继续测试其他控制命令"
    echo "  - python3 test_control.py left 0.5 2.0"
    echo "  - python3 test_control.py right 0.5 2.0"
    echo "  - python3 test_control.py spin 3.0"
else
    echo "⚠️  机器人似乎没有移动（距离变化 < $THRESHOLD 米）"
    echo ""
    echo "可能的原因："
    echo "1. 机器人可能被卡住或碰撞"
    echo "2. 控制命令可能没有正确发送"
    echo "3. 需要检查 Gazebo 中的机器人状态"
    echo ""
    echo "建议："
    echo "1. 查看 Gazebo 窗口，确认机器人是否可见"
    echo "2. 检查日志: ./view_logs.sh"
    echo "3. 尝试手动发布命令测试"
fi
echo "=========================================="

