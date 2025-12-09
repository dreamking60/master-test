#!/bin/bash
# Real-time attack connection monitor (READ-ONLY)
# Monitors and logs all connections to /cmd_vel topic
# NOTE: This script only OBSERVES and LOGS - it does NOT intercept or block attacks
# For actual security, use SROS2 (Secure ROS2)

echo "=========================================="
echo "Attack Connection Monitor (Read-Only)"
echo "=========================================="
echo ""
echo "⚠️  This is a MONITORING tool only"
echo "   - Observes and logs connections"
echo "   - Does NOT block or intercept attacks"
echo "   - For actual security, use SROS2"
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$PROJECT_ROOT/logs/monitoring"
mkdir -p "$LOG_DIR"

LOG_FILE="$LOG_DIR/attack_connections_$(date +%Y%m%d_%H%M%S).log"

echo "Monitoring /cmd_vel topic for attack connections..."
echo "Log file: $LOG_FILE"
echo ""
echo "Press Ctrl+C to stop monitoring"
echo "=========================================="
echo ""

# Function to log with timestamp
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Initial check
log_message "=== Monitoring Started ==="
log_message "Current ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
log_message "Current machine IP: $(hostname -I | awk '{print $1}')"
log_message ""

# Monitor /cmd_vel topic info (publishers)
PREVIOUS_PUBLISHERS=""

while true; do
    # Get current publishers
    CURRENT_PUBLISHERS=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
    PUBLISHER_NODES=$(ros2 topic info /cmd_vel 2>/dev/null | grep -A 10 "Publisher count:" | grep -E "^\s+/" | sed 's/^[[:space:]]*//')
    
    # Check if publishers changed
    if [ "$CURRENT_PUBLISHERS" != "$PREVIOUS_PUBLISHERS" ]; then
        if [ -n "$CURRENT_PUBLISHERS" ]; then
            log_message "⚠️  Publisher count changed: $CURRENT_PUBLISHERS"
            log_message "Publisher nodes:"
            echo "$PUBLISHER_NODES" | while read node; do
                if [ -n "$node" ]; then
                    log_message "  - $node"
                    
                    # Check if it's an unexpected node (attack indicator)
                    if [[ "$node" != *"robot_state_publisher"* ]] && [[ "$node" != *"teleop"* ]] && [[ "$node" != *"normal_operator"* ]] && [[ "$node" != *"controller"* ]]; then
                        log_message "🚨 ALERT: Unexpected publisher detected: $node"
                        log_message "   This might be an attack!"
                    fi
                fi
            done
            log_message ""
        fi
        PREVIOUS_PUBLISHERS="$CURRENT_PUBLISHERS"
    fi
    
    # Monitor message frequency (high frequency = potential attack)
    if command -v timeout &> /dev/null; then
        MSG_COUNT=$(timeout 2 ros2 topic hz /cmd_vel 2>/dev/null | grep "average rate:" | awk '{print $3}')
        if [ -n "$MSG_COUNT" ]; then
            FREQ=$(echo "$MSG_COUNT" | cut -d'.' -f1)
            if [ -n "$FREQ" ] && [ "$FREQ" -gt 30 ]; then
                log_message "⚠️  High message frequency detected: ${MSG_COUNT} Hz (potential attack)"
            fi
        fi
    fi
    
    # Check for new nodes
    CURRENT_NODES=$(ros2 node list 2>/dev/null | sort)
    if [ -n "$CURRENT_NODES" ]; then
        # Check for suspicious node names
        echo "$CURRENT_NODES" | grep -iE "(attack|injection|hack|malicious)" | while read node; do
            if [ -n "$node" ]; then
                log_message "🚨 ALERT: Suspicious node detected: $node"
            fi
        done
    fi
    
    sleep 2
done

