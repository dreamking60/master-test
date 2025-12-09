#!/bin/bash
# Run the injection attack experiment and record the robot's path
# This will save everything to data/ and results/ folders

echo "=========================================="
echo "Running Injection Attack Experiment"
echo "=========================================="
echo ""

# Make sure ROS2 is loaded
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi
export TURTLEBOT3_MODEL=burger

# Check if Gazebo is actually running
if ! ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "Error: Gazebo isn't running!"
    echo "Start it first with: ./scripts/setup/run_empty_world.sh"
    exit 1
fi

echo "Gazebo is running, good to go!"
echo ""

# Configuration
NORMAL_PATTERN="continuous"
NORMAL_FREQ=10
NORMAL_DURATION=60
ATTACK_TYPE="turn_left"
ATTACK_FREQ=50
ATTACK_DELAY=5
ATTACK_DURATION=15

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPERIMENTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DATA_DIR="$EXPERIMENTS_DIR/data"
RESULTS_DIR="$EXPERIMENTS_DIR/results"

# Create directories if they don't exist
mkdir -p "$DATA_DIR" "$RESULTS_DIR"

# Generate output filename with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_FILE="$DATA_DIR/trajectory_${TIMESTAMP}.csv"
PLOT_FILE="$RESULTS_DIR/trajectory_${TIMESTAMP}_plot.png"

echo "Experiment setup:"
echo "  Normal: robot goes straight ($NORMAL_FREQ Hz, $NORMAL_DURATION s)"
echo "  Attack: force left turn ($ATTACK_FREQ Hz, starts at ${ATTACK_DELAY}s, lasts ${ATTACK_DURATION}s)"
echo "  Data will be saved to: $OUTPUT_FILE"
echo ""
echo "Starting in 3 seconds..."
sleep 3
echo ""

# Start recording the robot's position
echo "[1/3] Starting trajectory recorder..."
python3 "$SCRIPT_DIR/record_trajectory.py" \
    --output "$OUTPUT_FILE" \
    --duration $((NORMAL_DURATION + 5)) \
    --plot > /tmp/trajectory_recorder.log 2>&1 &
RECORDER_PID=$!

echo "Recorder started (PID: $RECORDER_PID)"
sleep 1  # Give it a moment to initialize

# Start the normal controller
echo "[2/3] Starting normal controller (robot should go straight)..."
python3 "$SCRIPT_DIR/normal_controller.py" \
    --pattern "$NORMAL_PATTERN" \
    --frequency "$NORMAL_FREQ" \
    --duration "$NORMAL_DURATION" \
    --node-name "normal_operator" > /tmp/normal_controller.log 2>&1 &
NORMAL_PID=$!

echo "Normal controller running (PID: $NORMAL_PID)"
echo ""

# Wait a bit before attacking
echo "[3/3] Waiting ${ATTACK_DELAY}s before launching attack..."
for i in $(seq $ATTACK_DELAY -1 1); do
    echo -ne "   Attack in $i...\r"
    sleep 1
done
echo ""

echo ">>> ATTACK STARTING NOW! <<<"
echo "Watch Gazebo - robot should start turning left!"
echo ""

# Start attack (save logs to file)
python3 "$SCRIPT_DIR/injection_attack.py" \
    --attack-type "$ATTACK_TYPE" \
    --frequency "$ATTACK_FREQ" \
    --duration "$ATTACK_DURATION" \
    --angular-speed 0.5 \
    --node-name "injection_attacker" > /tmp/injection_attack.log 2>&1
ATTACK_EXIT_CODE=$?

echo ""
echo "=========================================="
echo "Attack finished!"
echo "=========================================="
echo ""
echo "Attack logs saved to: /tmp/injection_attack.log"
echo "View with: cat /tmp/injection_attack.log"
echo ""

# Wait for everything to finish
if kill -0 $NORMAL_PID 2>/dev/null; then
    echo "Waiting for normal controller to finish..."
    wait $NORMAL_PID 2>/dev/null
fi

echo "Waiting for recording to finish..."
wait $RECORDER_PID 2>/dev/null

echo ""
echo "=========================================="
echo "Results"
echo "=========================================="
echo ""
echo "Data saved to: $OUTPUT_FILE"
if [ -f "$PLOT_FILE" ]; then
    echo "Plot saved to: $PLOT_FILE"
fi
echo ""

if [ -f "$OUTPUT_FILE" ]; then
    TOTAL_POINTS=$(wc -l < "$OUTPUT_FILE")
    echo "Recorded $((TOTAL_POINTS - 1)) data points ($(du -h "$OUTPUT_FILE" | cut -f1))"
fi
echo ""
echo "To analyze:"
echo "  cd $SCRIPT_DIR"
echo "  python3 analyze_trajectory.py $OUTPUT_FILE"
echo ""
echo "What to look for:"
echo "  - Before attack: straight line (Y stays near 0)"
echo "  - During attack: should curve left (Y increases)"
echo "  - After attack: back to straight line"
echo ""

