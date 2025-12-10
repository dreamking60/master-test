#!/bin/bash
# Test if robot can move - diagnostic script

echo "=========================================="
echo "Robot Movement Test"
echo "=========================================="
echo ""

# Setup ROS2
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Clear SROS2 if incomplete
if [ -n "$ROS_SECURITY_KEYSTORE" ] && [ ! -d "$ROS_SECURITY_KEYSTORE/enclaves" ]; then
    echo "Clearing incomplete SROS2 configuration..."
    unset ROS_SECURITY_KEYSTORE
    unset ROS_SECURITY_ENABLE
    unset ROS_SECURITY_STRATEGY
fi

echo "Step 1: Check Gazebo..."
if pgrep -f "gz sim" > /dev/null; then
    echo "✅ Gazebo is running"
else
    echo "❌ Gazebo is NOT running"
    echo "   Start it first: ./scripts/setup/restart_gazebo.sh"
    exit 1
fi

echo ""
echo "Step 2: Check /cmd_vel topic..."
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "✅ /cmd_vel topic exists"
    
    # Check publishers
    PUBLISHERS=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
    echo "   Publishers: $PUBLISHERS"
else
    echo "❌ /cmd_vel topic NOT found"
    echo "   Robot may not be spawned"
    exit 1
fi

echo ""
echo "Step 3: Test sending command..."
echo "Sending forward command (0.2 m/s) for 3 seconds..."

# Send test command
timeout 3 ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    --rate 10 &

PUB_PID=$!

sleep 3
kill $PUB_PID 2>/dev/null

echo ""
echo "Step 4: Check robot position..."
sleep 1

INITIAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)

if [ -n "$INITIAL_POSE" ]; then
    INITIAL_X=$(echo "$INITIAL_POSE" | grep "x:" | awk '{print $2}')
    INITIAL_Y=$(echo "$INITIAL_POSE" | grep "y:" | awk '{print $2}')
    echo "Robot position: x=$INITIAL_X, y=$INITIAL_Y"
    
    echo ""
    echo "Send another command and check if position changes..."
    timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
        --rate 10 > /dev/null 2>&1 &
    
    sleep 2
    
    FINAL_POSE=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep -E "x:|y:" | head -2)
    if [ -n "$FINAL_POSE" ]; then
        FINAL_X=$(echo "$FINAL_POSE" | grep "x:" | awk '{print $2}')
        FINAL_Y=$(echo "$FINAL_POSE" | grep "y:" | awk '{print $2}')
        
        DELTA_X=$(python3 -c "print($FINAL_X - $INITIAL_X)" 2>/dev/null)
        DELTA_Y=$(python3 -c "print($FINAL_Y - $INITIAL_Y)" 2>/dev/null)
        DISTANCE=$(python3 -c "import math; print(math.sqrt($DELTA_X**2 + $DELTA_Y**2))" 2>/dev/null)
        
        echo "New position: x=$FINAL_X, y=$FINAL_Y"
        echo "Movement: $DISTANCE m"
        
        if (( $(echo "$DISTANCE > 0.01" | bc -l) )); then
            echo "✅ Robot is moving!"
        else
            echo "⚠️  Robot position didn't change much"
            echo "   Possible issues:"
            echo "   - Robot may be stuck"
            echo "   - Commands not being processed"
            echo "   - Check Gazebo window"
        fi
    fi
else
    echo "⚠️  Cannot get robot position"
fi

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="

