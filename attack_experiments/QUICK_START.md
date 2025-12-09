# Quick Start Guide

## Step-by-Step: Run Experiment with Data Recording

### Step 1: Start Gazebo (Empty World)

**In Terminal 1:**
```bash
cd /home/stevenchen/master-test
./scripts/setup/run_empty_world.sh
```

Wait for Gazebo to fully load (you should see the robot on an empty plane).

### Step 2: Run Experiment with Recording

**In Terminal 2:**
```bash
cd /home/stevenchen/master-test/attack_experiments/scripts
./run_experiment_with_recording.sh
```

This will:
- Start normal controller (robot moves straight forward)
- Wait 5 seconds
- Launch attack (robot turns left)
- Record trajectory to `../data/trajectory_*.csv`
- Generate plot in `../results/trajectory_*_plot.png`

### Step 3: Analyze Results

After experiment completes:
```bash
cd /home/stevenchen/master-test/attack_experiments/scripts
python3 analyze_trajectory.py ../data/trajectory_YYYYMMDD_HHMMSS.csv
```

## Alternative: Quick Test (No Recording)

If you just want to see the attack without recording data:
```bash
cd /home/stevenchen/master-test/attack_experiments/scripts
./quick_test.sh
```

This runs the experiment without saving trajectory data.

