#!/bin/bash
# Test rate limiting defense mechanism
# This test demonstrates message frequency monitoring

echo "=========================================="
echo "Defense Test: Rate Limiting"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This test demonstrates rate limiting as a defense mechanism."
echo ""
echo "High-frequency attacks can be detected by monitoring message rates."
echo ""

read -p "Press Enter to start monitoring..."

echo "=========================================="
echo "Monitoring /cmd_vel Message Rate"
echo "=========================================="
echo ""
echo "Normal operation: ~10 Hz (from normal controller)"
echo "Attack: 50+ Hz (high frequency to override)"
echo ""
echo "Starting rate monitoring..."
echo "Press Ctrl+C to stop"
echo ""

# Monitor message rate
while true; do
    rate=$(timeout 3 ros2 topic hz /cmd_vel 2>/dev/null | grep "average rate" | awk '{print $3}')
    
    if [ -n "$rate" ]; then
        rate_num=$(echo "$rate" | sed 's/Hz//')
        rate_int=${rate_num%.*}
        
        timestamp=$(date +%H:%M:%S)
        
        if [ "$rate_int" -gt 40 ]; then
            echo "[$timestamp] ⚠️  HIGH RATE DETECTED: $rate (Possible attack!)"
        elif [ "$rate_int" -gt 20 ]; then
            echo "[$timestamp] ⚠️  Elevated rate: $rate (Monitor closely)"
        else
            echo "[$timestamp] Normal rate: $rate"
        fi
    else
        echo "[$(date +%H:%M:%S)] No messages (topic may not exist)"
    fi
    
    sleep 2
done

