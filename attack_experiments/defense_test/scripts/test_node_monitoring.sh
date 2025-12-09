#!/bin/bash
# Test node monitoring defense mechanism
# This test demonstrates how to detect unexpected publishers

echo "=========================================="
echo "Defense Test: Node Monitoring"
echo "=========================================="
echo ""

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

echo "This test demonstrates how to monitor and detect malicious nodes."
echo ""

read -p "Press Enter to start monitoring..."

echo "=========================================="
echo "Monitoring /cmd_vel Publishers"
echo "=========================================="
echo ""
echo "Baseline: Check current publishers"
echo ""

# Get baseline publishers
echo "Current /cmd_vel topic info:"
ros2 topic info /cmd_vel 2>/dev/null || echo "Topic not found (Gazebo may not be running)"
echo ""

echo "Expected publishers (normal operation):"
echo "  - turtlebot3_teleop (if using teleop_keyboard)"
echo "  - normal_controller (if using our test scripts)"
echo ""

echo "=========================================="
echo "Starting Continuous Monitoring"
echo "=========================================="
echo ""
echo "Monitoring for unexpected publishers..."
echo "Press Ctrl+C to stop monitoring"
echo ""

# Monitor publishers
while true; do
    publishers=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
    
    if [ -n "$publishers" ] && [ "$publishers" -gt 0 ]; then
        echo "[$(date +%H:%M:%S)] Publishers detected: $publishers"
        
        # List actual publisher nodes
        ros2 topic info /cmd_vel 2>/dev/null | grep -A 10 "Publisher" | grep "node name" | while read line; do
            node_name=$(echo "$line" | awk '{print $NF}')
            echo "  - $node_name"
            
            # Check for suspicious node names
            if [[ "$node_name" == *"attacker"* ]] || [[ "$node_name" == *"injection"* ]] || [[ "$node_name" == *"malicious"* ]]; then
                echo "    ⚠️  SUSPICIOUS NODE DETECTED!"
            fi
        done
    else
        echo "[$(date +%H:%M:%S)] No publishers"
    fi
    
    sleep 2
done

