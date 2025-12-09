#!/bin/bash
# ROS2 log viewer tool

# Get project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
OUTPUT_LOG="$PROJECT_ROOT/test.log"

echo "=========================================="
echo "ROS2 Log Viewer"
echo "=========================================="
echo ""

# Check log directory
if [ -d "$LOG_DIR" ]; then
    echo "✅ ROS2 log directory: $LOG_DIR"
    LOG_COUNT=$(find "$LOG_DIR" -name "*.log" 2>/dev/null | wc -l)
    echo "   Found $LOG_COUNT log file(s)"
else
    echo "⚠️  Log directory does not exist: $LOG_DIR"
fi
echo ""

# Check startup script output log
if [ -f "$OUTPUT_LOG" ]; then
    LOG_SIZE=$(du -h "$OUTPUT_LOG" | cut -f1)
    echo "✅ Startup script log: $OUTPUT_LOG ($LOG_SIZE)"
else
    echo "⚠️  Startup script log does not exist: $OUTPUT_LOG"
fi
echo ""

# Display menu
echo "Please select an operation:"
echo "1) View latest ROS2 log file"
echo "2) View startup script output (test.log)"
echo "3) Search for errors"
echo "4) Search for warnings"
echo "5) View all log file list"
echo "6) Monitor logs in real-time (tail -f)"
echo "7) Use rqt_console (GUI)"
echo "8) Clean all logs"
echo "0) Exit"
echo ""

read -p "Enter option [0-8]: " choice

case $choice in
    1)
        if [ -d "$LOG_DIR" ]; then
            LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            if [ -n "$LATEST_LOG" ]; then
                echo "Latest log file: $LATEST_LOG"
                echo "Showing last 50 lines:"
                echo "----------------------------------------"
                tail -50 "$LATEST_LOG"
            else
                echo "No log files found"
            fi
        fi
        ;;
    2)
        if [ -f "$OUTPUT_LOG" ]; then
            echo "Showing last 50 lines:"
            echo "----------------------------------------"
            tail -50 "$OUTPUT_LOG"
        else
            echo "Log file does not exist"
        fi
        ;;
    3)
        echo "Searching for errors..."
        if [ -d "$LOG_DIR" ]; then
            grep -i "error" "$LOG_DIR"/*.log 2>/dev/null | tail -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- In test.log ---"
            grep -i "error" "$OUTPUT_LOG" | tail -20
        fi
        ;;
    4)
        echo "Searching for warnings..."
        if [ -d "$LOG_DIR" ]; then
            grep -i "warn" "$LOG_DIR"/*.log 2>/dev/null | tail -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- In test.log ---"
            grep -i "warn" "$OUTPUT_LOG" | tail -20
        fi
        ;;
    5)
        echo "All log files:"
        if [ -d "$LOG_DIR" ]; then
            find "$LOG_DIR" -name "*.log" -type f -exec ls -lh {} + | head -20
        fi
        if [ -f "$OUTPUT_LOG" ]; then
            echo "--- test.log ---"
            ls -lh "$OUTPUT_LOG"
        fi
        ;;
    6)
        echo "Monitoring logs in real-time (Press Ctrl+C to exit)..."
        if [ -d "$LOG_DIR" ]; then
            LATEST_LOG=$(find "$LOG_DIR" -name "*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            if [ -n "$LATEST_LOG" ]; then
                tail -f "$LATEST_LOG"
            else
                echo "No log files found"
            fi
        elif [ -f "$OUTPUT_LOG" ]; then
            tail -f "$OUTPUT_LOG"
        else
            echo "No log files found"
        fi
        ;;
    7)
        echo "Starting rqt_console..."
        if command -v ros2 &> /dev/null; then
            ros2 run rqt_console rqt_console &
        else
            echo "Error: ros2 command not found, please source ROS2 environment first"
        fi
        ;;
    8)
        read -p "Confirm to delete all logs? (y/N): " confirm
        if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
            if [ -d "$LOG_DIR" ]; then
                rm -rf "$LOG_DIR"/*
                echo "Cleaned ROS2 log directory"
            fi
            if [ -f "$OUTPUT_LOG" ]; then
                rm -f "$OUTPUT_LOG"
                echo "Deleted test.log"
            fi
            echo "Log cleanup completed"
        else
            echo "Cancelled"
        fi
        ;;
    0)
        echo "Exit"
        exit 0
        ;;
    *)
        echo "Invalid option"
        ;;
esac
