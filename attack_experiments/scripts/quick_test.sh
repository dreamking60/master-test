#!/bin/bash
# Quick test - just runs the experiment without recording data
# Good for checking if everything works before running the full experiment

echo "=========================================="
echo "Quick Injection Attack Test"
echo "=========================================="
echo ""

# Load ROS2
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Make sure Gazebo is running
if ! ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "Error: Gazebo isn't running!"
    echo "Start it first:"
    echo "  ./scripts/setup/run_empty_world.sh  (empty world - easier to see)"
    echo "  or"
    echo "  ./scripts/setup/run.sh  (world with obstacles)"
    exit 1
fi

echo "Gazebo is running, good!"
echo ""

# Settings
NORMAL_PATTERN="continuous"  # Robot goes straight
NORMAL_FREQ=10
NORMAL_DURATION=60
ATTACK_TYPE="turn_left"  # Attack makes it turn left
ATTACK_FREQ=50
ATTACK_DELAY=5
ATTACK_DURATION=15

echo "Settings:"
echo "  Normal: straight @ $NORMAL_FREQ Hz for $NORMAL_DURATION s"
echo "  Attack: left turn @ $ATTACK_FREQ Hz, starts at ${ATTACK_DELAY}s, lasts ${ATTACK_DURATION}s"
echo ""
echo "What should happen:"
echo "  - Normal: robot goes straight"
echo "  - Attack: robot turns left (if it works)"
echo ""
echo "Starting in 3 seconds..."
sleep 3
echo ""

# Get script location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Start normal controller
echo "[1/2] Starting normal controller..."
python3 "$SCRIPT_DIR/normal_controller.py" \
    --pattern "$NORMAL_PATTERN" \
    --frequency "$NORMAL_FREQ" \
    --duration "$NORMAL_DURATION" \
    --node-name "normal_operator" > /tmp/normal_controller.log 2>&1 &
NORMAL_PID=$!

echo "Normal controller running (PID: $NORMAL_PID)"
echo "Robot should be going straight now"
echo ""

# Wait a bit before attacking
echo "[2/2] Waiting ${ATTACK_DELAY}s before attack..."
for i in $(seq $ATTACK_DELAY -1 1); do
    echo -ne "   Attack in $i...\r"
    sleep 1
done
echo ""

echo ">>> ATTACK STARTING! <<<"
echo "Watch Gazebo - robot should turn left!"
echo ""

# Run the attack
python3 "$SCRIPT_DIR/injection_attack.py" \
    --attack-type "$ATTACK_TYPE" \
    --frequency "$ATTACK_FREQ" \
    --duration "$ATTACK_DURATION" \
    --angular-speed 0.5 \
    --node-name "injection_attacker"

echo ""
echo "=========================================="
echo "Done!"
echo "=========================================="
echo ""

# Clean up
if kill -0 $NORMAL_PID 2>/dev/null; then
    echo "Stopping normal controller..."
    kill $NORMAL_PID 2>/dev/null
    wait $NORMAL_PID 2>/dev/null
    echo "Stopped"
fi

echo ""
echo "Summary:"
echo "  - Normal: straight @ $NORMAL_FREQ Hz"
echo "  - Attack: left turn @ $ATTACK_FREQ Hz"
echo "  - Check Gazebo to see if the attack worked"
echo ""
echo "Logs: cat /tmp/normal_controller.log"

