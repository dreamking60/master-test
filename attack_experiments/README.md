# Attack Experiments

This folder has all the scripts and docs for testing command injection attacks on ROS2 robots.

## Folder Structure

```
attack_experiments/
├── README.md                    # You are here
├── STRUCTURE.md                  # More details about the structure
│
├── scripts/                      # All the scripts
│   ├── normal_controller.py      # Simulates a normal user (robot goes straight)
│   ├── injection_attack.py       # The attack script (forces left turn)
│   ├── record_trajectory.py      # Records where the robot goes
│   ├── analyze_trajectory.py     # Analyzes the recorded data
│   ├── quick_test.sh             # Quick test without recording
│   ├── run_injection_experiment.sh      # Interactive version
│   ├── run_experiment_with_recording.sh # Full experiment with data recording
│   └── deploy_to_real_robot.sh   # How to test on a real robot
│
├── docs/                         # Documentation
│   ├── README.md                 # Docs index
│   ├── README_EN.md              # English docs
│   ├── INJECTION_EXPERIMENT.md   # How to run the experiment
│   ├── LOGIC_AND_SUCCESS_CRITERIA.md  # How the scripts work
│   ├── EXPERIMENT_RESULTS.md     # How to interpret results
│   └── README_EMPTY_WORLD.md     # Using the empty world
│
├── data/                         # CSV files with trajectory data
│   └── trajectory_*.csv           # Robot position over time
│
└── results/                      # Plots and analysis
    └── trajectory_*_plot.png      # Visualizations
```

## Quick Start

### 1. Quick Test (no recording)

```bash
cd attack_experiments/scripts
./quick_test.sh
```

### 2. Full Experiment (with data recording)

```bash
cd attack_experiments/scripts
./run_experiment_with_recording.sh
```

This will:
- Record where the robot goes
- Save everything to `data/trajectory_*.csv`
- Make a plot in `results/trajectory_*_plot.png`

### 3. Look at the Results

```bash
cd attack_experiments/scripts
python3 analyze_trajectory.py ../data/trajectory_YYYYMMDD_HHMMSS.csv
```

Use whatever filename the script printed out.

## Documentation

Everything is in `docs/`:

- **README_EN.md** - Full guide to the attack experiments
- **INJECTION_EXPERIMENT.md** - How to run the injection experiment
- **LOGIC_AND_SUCCESS_CRITERIA.md** - How the scripts work and what success looks like
- **EXPERIMENT_RESULTS.md** - How to record and analyze results
- **README_EMPTY_WORLD.md** - Using the empty world (easier to see what's happening)

## Data Files

### `data/` folder
- All the CSV files with robot positions go here
- Named like `trajectory_YYYYMMDD_HHMMSS.csv`
- Has: time, position (x,y), angle, velocities

### `results/` folder
- Plots (PNG images)
- Analysis reports
- Basically anything that shows what happened

### Cleaning up old files
```bash
# Delete data older than 7 days
find data/ -name "*.csv" -mtime +7 -delete
find results/ -name "*.png" -mtime +7 -delete
```

## Scripts

### Main Scripts
- `normal_controller.py` - Normal user (robot goes straight)
- `injection_attack.py` - The attack (forces left turn)
- `record_trajectory.py` - Records where the robot goes
- `analyze_trajectory.py` - Looks at the data and makes a report

### Experiment Scripts
- `quick_test.sh` - Quick test without recording
- `run_injection_experiment.sh` - Interactive version
- `run_experiment_with_recording.sh` - Full experiment with data recording

## Important Notes

1. Run scripts from `scripts/` or use full paths
2. Data goes to `data/` automatically
3. Results go to `results/` automatically
4. Paths are relative to the experiment folder

## Examples

### Quick Test
```bash
cd attack_experiments/scripts
./quick_test.sh
```

### Full Experiment
```bash
cd attack_experiments/scripts
./run_experiment_with_recording.sh
```

### Manual (3 terminals)
```bash
cd attack_experiments/scripts

# Terminal 1: Normal controller
python3 normal_controller.py --pattern continuous --frequency 10 --duration 60

# Terminal 2: Attack (wait 5 seconds first)
sleep 5
python3 injection_attack.py --attack-type turn_left --frequency 50 --duration 15

# Terminal 3: Record trajectory
python3 record_trajectory.py --output ../data/trajectory.csv --duration 60 --plot
```

### Analyze
```bash
cd attack_experiments/scripts
python3 analyze_trajectory.py ../data/trajectory_YYYYMMDD_HHMMSS.csv
```

