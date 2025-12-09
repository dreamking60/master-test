#!/bin/bash
# Quick tool to view attack logs

echo "=========================================="
echo "Attack Log Viewer"
echo "=========================================="
echo ""

# Check temporary log file
TMP_LOG="/tmp/injection_attack.log"
if [ -f "$TMP_LOG" ]; then
    echo "✅ Found temporary attack log: $TMP_LOG"
    LOG_SIZE=$(du -h "$TMP_LOG" | cut -f1)
    echo "   File size: $LOG_SIZE"
    echo ""
    echo "Last 30 lines:"
    echo "----------------------------------------"
    tail -30 "$TMP_LOG"
    echo ""
    echo "----------------------------------------"
    echo "View full log: cat $TMP_LOG"
    echo ""
else
    echo "⚠️  Temporary log file does not exist: $TMP_LOG"
    echo ""
fi

# Check ROS2 log directory
ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -d "$ROS_LOG_DIR" ]; then
    echo "✅ ROS2 log directory: $ROS_LOG_DIR"
    
    # Find log files containing attack information
    ATTACK_LOGS=$(find "$ROS_LOG_DIR" -name "*.log" -type f -exec grep -l "injection_attacker\|ATTACK MODE" {} \; 2>/dev/null)
    
    if [ -n "$ATTACK_LOGS" ]; then
        LOG_COUNT=$(echo "$ATTACK_LOGS" | wc -l)
        echo "   Found $LOG_COUNT log file(s) containing attack information"
        echo ""
        
        # Display latest attack log
        LATEST_ATTACK_LOG=$(echo "$ATTACK_LOGS" | xargs ls -t | head -1)
        if [ -n "$LATEST_ATTACK_LOG" ]; then
            echo "📋 Latest attack log: $LATEST_ATTACK_LOG"
            echo "----------------------------------------"
            grep -i "attack\|injection_attacker" "$LATEST_ATTACK_LOG" | tail -20
            echo ""
            echo "----------------------------------------"
            echo "View full file: cat $LATEST_ATTACK_LOG"
            echo ""
        fi
        
        # Display all attack log files
        echo "All attack log files:"
        echo "$ATTACK_LOGS" | while read log; do
            if [ -n "$log" ]; then
                SIZE=$(du -h "$log" | cut -f1)
                MTIME=$(stat -c %y "$log" 2>/dev/null | cut -d' ' -f1-2)
                echo "  - $log ($SIZE, $MTIME)"
            fi
        done
        echo ""
    else
        echo "⚠️  No log files containing attack information found"
        echo ""
    fi
else
    echo "⚠️  ROS2 log directory does not exist: $ROS_LOG_DIR"
    echo ""
fi

# Provide quick commands
echo "=========================================="
echo "Quick Commands"
echo "=========================================="
echo ""
echo "1. View temporary attack log:"
echo "   cat /tmp/injection_attack.log"
echo ""
echo "2. Search all attack logs:"
echo "   grep -i 'attack\|injection_attacker' ~/.ros/log/*.log"
echo ""
echo "3. Monitor attack logs in real-time:"
echo "   tail -f ~/.ros/log/latest/*.log | grep -i attack"
echo ""
echo "4. Use rqt_console (GUI):"
echo "   ros2 run rqt_console rqt_console"
echo ""
echo "5. View detailed instructions:"
echo "   cat ../sros2_test/HOW_TO_VIEW_ATTACK_LOGS.md"
echo ""
