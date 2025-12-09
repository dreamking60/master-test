#!/bin/bash
# ROS2 日志查看工具

# 获取项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
OUTPUT_LOG="$PROJECT_ROOT/test.log"

echo "=========================================="
echo "ROS2 日志查看工具"
echo "=========================================="
echo ""

# 检查日志目录
if [ -d "$LOG_DIR" ]; then
    echo "✅ ROS2 日志目录: $LOG_DIR"
    LOG_COUNT=$(find "$LOG_DIR" -name "*.log" 2>/dev/null | wc -l)
    echo "   找到 $LOG_COUNT 个日志文件"
else
    echo "⚠️  日志目录不存在: $LOG_DIR"
fi
echo ""

# 检查启动脚本的输出日志
if [ -f "$OUTPUT_LOG" ]; then
    LOG_SIZE=$(du -h "$OUTPUT_LOG" | cut -f1)
    echo "✅ 启动脚本日志: $OUTPUT_LOG ($LOG_SIZE)"
else
    echo "⚠️  启动脚本日志不存在: $OUTPUT_LOG"
fi
echo ""

# 显示菜单
echo "请选择要执行的操作："
echo "1) 查看最新的 ROS2 日志文件"
echo "2) 查看启动脚本输出 (test.log)"
echo "3) 搜索错误信息"
echo "4) 搜索警告信息"
echo "5) 查看所有日志文件列表"
echo "6) 实时监控日志（tail -f）"
echo "7) 使用 rqt_console（图形界面）"
echo "8) 清理所有日志"
echo "0) 退出"
echo ""

read -p "请输入选项 [0-8]: " choice

case $choice in
    1)
        if [ -d "$LOG_DIR" ]; then
            LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            if [ -n "$LATEST_LOG" ]; then
                echo "最新的日志文件: $LATEST_LOG"
                echo "显示最后 50 行："
                echo "----------------------------------------"
                tail -50 "$LATEST_LOG"
            else
                echo "未找到日志文件"
            fi
        fi
        ;;
    2)
        if [ -f "$OUTPUT_LOG" ]; then
            echo "显示最后 50 行："
            echo "----------------------------------------"
            tail -50 "$OUTPUT_LOG"
        else
            echo "日志文件不存在"
        fi
        ;;
    3)
        echo "搜索错误信息..."
        if [ -d "$LOG_DIR" ]; then
            grep -i "error" "$LOG_DIR"/*.log 2>/dev/null | tail -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- 在 test.log 中 ---"
            grep -i "error" "$OUTPUT_LOG" | tail -20
        fi
        ;;
    4)
        echo "搜索警告信息..."
        if [ -d "$LOG_DIR" ]; then
            grep -i "warn" "$LOG_DIR"/*.log 2>/dev/null | tail -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- 在 test.log 中 ---"
            grep -i "warn" "$OUTPUT_LOG" | tail -20
        fi
        ;;
    5)
        echo "所有日志文件："
        if [ -d "$LOG_DIR" ]; then
            find "$LOG_DIR" -name "*.log" -type f -exec ls -lh {} + | head -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- test.log ---"
            ls -lh "$OUTPUT_LOG"
        fi
        ;;
    6)
        echo "实时监控日志（按 Ctrl+C 退出）..."
        if [ -d "$LOG_DIR" ]; then
            LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            if [ -n "$LATEST_LOG" ]; then
                tail -f "$LATEST_LOG"
            else
                echo "未找到日志文件"
            fi
        elif [ -f "$OUTPUT_LOG" ]; then
            tail -f "$OUTPUT_LOG"
        else
            echo "未找到日志文件"
        fi
        ;;
    7)
        echo "启动 rqt_console..."
        if command -v ros2 &> /dev/null; then
            ros2 run rqt_console rqt_console &
        else
            echo "错误: 未找到 ros2 命令，请先 source ROS2 环境"
        fi
        ;;
    8)
        read -p "确认要删除所有日志吗？(y/N): " confirm
        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            if [ -d "$LOG_DIR" ]; then
                rm -rf "$LOG_DIR"/*
                echo "已清理 ROS2 日志目录"
            fi
            if [ -f "$OUTPUT_LOG" ]; then
                rm -f "$OUTPUT_LOG"
                echo "已删除 test.log"
            fi
            echo "日志清理完成"
        else
            echo "已取消"
        fi
        ;;
    0)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选项"
        ;;
esac

