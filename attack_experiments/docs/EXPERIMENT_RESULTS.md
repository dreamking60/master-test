# Experiment Results Documentation

## How to Record and Analyze Trajectory

### Step 1: Run Experiment with Recording

```bash
cd attack_experiments
./run_experiment_with_recording.sh
```

This will:
- Start trajectory recorder
- Run normal controller (continuous forward)
- Launch attack (turn left) after 5 seconds
- Save trajectory to CSV file with timestamp
- Generate trajectory plot (PNG image)

### Step 2: Analyze Results

After experiment completes, analyze the trajectory:

```bash
python3 analyze_trajectory.py trajectory_YYYYMMDD_HHMMSS.csv
```

Or with custom attack timing:

```bash
python3 analyze_trajectory.py trajectory.csv --attack-start 5 --attack-duration 15
```

### Step 3: Review Output Files

1. **CSV File** (`trajectory_*.csv`):
   - Contains all position data
   - Columns: timestamp, relative_time, x, y, z, yaw, linear_x, linear_y, angular_z
   - Can be opened in Excel, Python, or any data analysis tool

2. **Plot File** (`trajectory_*_plot.png`):
   - Visual representation of trajectory
   - Top view (x-y plot) showing robot path
   - Position vs time graph

3. **Analysis Report** (from analyze_trajectory.py):
   - Detailed statistics for each period
   - Attack success assessment
   - Quantitative metrics

## Expected Results

### Before Attack (0-5 seconds)
- **Behavior**: Robot moves straight forward
- **X Position**: Increases steadily
- **Y Position**: Stays near 0 (minimal lateral movement)
- **Angular Velocity**: Near 0 (no turning)
- **Yaw Change**: Minimal

### During Attack (5-20 seconds)
- **Behavior**: Robot turns left (if attack successful)
- **X Position**: Still increases but slower
- **Y Position**: Increases (lateral movement to the left)
- **Angular Velocity**: Positive (left turn)
- **Yaw Change**: Significant increase

### After Attack (20+ seconds)
- **Behavior**: Robot returns to straight forward
- **X Position**: Increases steadily again
- **Y Position**: Stabilizes
- **Angular Velocity**: Returns to near 0

## Success Criteria

Attack is considered **successful** if:

1. ✅ **Angular Velocity Increase**: Average angular velocity during attack > 0.1 rad/s
2. ✅ **Yaw Change**: Yaw change during attack > 2x before attack
3. ✅ **Lateral Movement**: Y position change during attack > 2x before attack

## Example Analysis Output

```
======================================================================
INJECTION ATTACK EXPERIMENT - TRAJECTORY ANALYSIS REPORT
======================================================================
Analysis Date: 2024-12-08 16:30:00
Total Experiment Duration: 60.00 seconds
Attack Period: 5.0s - 20.0s (15.0s)

----------------------------------------------------------------------
BEFORE ATTACK (Normal Operation)
----------------------------------------------------------------------
  Duration: 5.00 seconds
  Start Position: (0.000, 0.000)
  End Position: (1.000, 0.002)
  Total Distance Traveled: 1.000 m
  Displacement: 1.000 m
  Average Linear Velocity: 0.200 m/s
  Average Angular Velocity: 0.001 rad/s
  Yaw Change: 0.005 rad (0.3°)
  X Range: [0.000, 1.000]
  Y Range: [-0.001, 0.002]

----------------------------------------------------------------------
DURING ATTACK
----------------------------------------------------------------------
  Duration: 15.00 seconds
  Start Position: (1.000, 0.002)
  End Position: (3.500, 0.850)
  Total Distance Traveled: 2.650 m
  Displacement: 2.550 m
  Average Linear Velocity: 0.180 m/s
  Average Angular Velocity: 0.450 rad/s
  Yaw Change: 0.785 rad (45.0°)
  X Range: [1.000, 3.500]
  Y Range: [0.002, 0.850]

======================================================================
ATTACK SUCCESS ANALYSIS
======================================================================
Angular Velocity Change: 0.449 rad/s
  Before: 0.001 rad/s
  During: 0.450 rad/s

Yaw Change Comparison:
  Before Attack: 0.005 rad (0.3°)
  During Attack: 0.785 rad (45.0°)

Y Position Change (Lateral Movement):
  Before Attack: 0.002 m
  During Attack: 0.848 m

SUCCESS CRITERIA:
  ✅ Angular velocity increased significantly
  ✅ Yaw change increased (robot turned more)
  ✅ Lateral movement increased (robot turned left)

🎯 ATTACK SUCCESSFUL: Robot behavior changed significantly during attack
```

## Using Results in Report

### Quantitative Data
- Use CSV file for detailed numerical analysis
- Extract specific metrics (displacement, velocity, etc.)
- Compare before/during/after periods

### Visual Evidence
- Include trajectory plot in report
- Annotate plot to show attack period
- Highlight behavior changes

### Statistical Analysis
- Use analysis report for summary statistics
- Document success criteria results
- Note any anomalies or unexpected behavior

## Tips for Best Results

1. **Use Empty World**: Better visibility of trajectory
2. **Record Full Duration**: Include before, during, and after periods
3. **Multiple Runs**: Run experiment multiple times for consistency
4. **Document Parameters**: Note attack frequency, duration, etc.
5. **Compare Scenarios**: Try different attack types and compare results

