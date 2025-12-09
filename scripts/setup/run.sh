#!/bin/bash
# 设置日志目录
# 获取脚本所在目录的父目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export ROS_LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$PROJECT_ROOT/logs"

# 启动 Gazebo 仿真，输出重定向到项目根目录的 test.log
cd "$PROJECT_ROOT"
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > test.log 2>&1
