# Project Structure Documentation

## 📂 Directory Structure

```
master-test/
├── README.md                    # Main project documentation
├── PROJECT_STRUCTURE.md         # This file (project structure documentation)
│
├── attack_experiments/         # Attack experiments directory
│   ├── README.md               # Attack experiment index
│   ├── README_CN.md           # Chinese version attack experiment documentation
│   ├── README_EN.md           # English version attack experiment documentation
│   ├── attack_cmd_vel.py      # Original attack script (compatible)
│   ├── attack_cmd_vel_CN.py   # Chinese version attack script
│   ├── attack_cmd_vel_EN.py   # English version attack script
│   └── deploy_to_real_robot.sh # Real robot deployment script
│
├── scripts/                    # Scripts directory
│   ├── setup/                  # Setup and run scripts
│   │   ├── run.sh              # Launch Gazebo simulation
│   │   └── keyboard.sh         # Keyboard control script
│   └── testing/                # Testing and diagnostic scripts
│       ├── check_status.sh     # System status check
│       ├── diagnose_teleop.sh  # Keyboard control diagnosis
│       ├── test_control.py     # Python control test script
│       ├── test_movement.sh   # Movement test script
│       └── view_logs.sh        # Log viewer tool
│
├── docs/                        # Documentation directory
│   ├── env.md                  # Environment setup guide (complete version)
│   └── cmd_vel_injection_attack.md # Attack experiment documentation (old version, migrated to attack_experiments/)
│
└── logs/                        # Logs directory
    ├── test.log                # Launch script output log
    └── [ROS2 log files]          # ROS2 auto-generated logs
```

## 📝 File Category Description

### Attack Experiments (`attack_experiments/`)
All files related to attack experiments, including:
- Chinese and English documentation
- Chinese and English attack scripts
- Real robot deployment scripts

### Setup Scripts (`scripts/setup/`)
Scripts for launching and running simulation environments:
- `run.sh` - Launch Gazebo simulation, automatically configure log directory
- `keyboard.sh` - Launch keyboard control node

### Testing Scripts (`scripts/testing/`)
Scripts for testing, diagnosis, and monitoring:
- `check_status.sh` - Check ROS2 system status
- `diagnose_teleop.sh` - Diagnose keyboard control issues
- `test_control.py` - Python control test (supports forward, backward, rotation, etc.)
- `test_movement.sh` - Automatically test robot movement
- `view_logs.sh` - Interactive log viewer tool

### Documentation (`docs/`)
Project documentation:
- `env.md` - Complete environment setup guide (from virtual machine to ROS2 installation)
- `cmd_vel_injection_attack.md` - Attack experiment documentation (old version, new version in attack_experiments/)

## 🚀 Usage Guide

### Quick Start

1. **Environment Setup**: Follow `docs/env.md` to set up the environment
2. **Launch Simulation**: `./scripts/setup/run.sh`
3. **Test Control**: `python3 scripts/testing/test_control.py forward 0.2 2.0`
4. **Run Attack**: `cd attack_experiments && python3 attack_cmd_vel_EN.py --attack forward --speed 0.5`

### Path Notes

All scripts are designed to run from the project root directory, or use relative paths. Scripts internally handle path issues automatically.

## 🔄 File Migration Notes

The following files have been migrated from the root directory to corresponding subdirectories:

- `run.sh` → `scripts/setup/run.sh`
- `keyboard.sh` → `scripts/setup/keyboard.sh`
- `check_status.sh` → `scripts/testing/check_status.sh`
- `diagnose_teleop.sh` → `scripts/testing/diagnose_teleop.sh`
- `test_control.py` → `scripts/testing/test_control.py`
- `test_movement.sh` → `scripts/testing/test_movement.sh`
- `view_logs.sh` → `scripts/testing/view_logs.sh`
- `env.md` → `docs/env.md`
- `cmd_vel_injection_attack.md` → `docs/cmd_vel_injection_attack.md`
- `attack_cmd_vel.py` → `attack_experiments/attack_cmd_vel.py`
- `deploy_to_real_robot.sh` → `attack_experiments/deploy_to_real_robot.sh`

## 📌 Important Notes

1. When running scripts, it is recommended to execute from the project root directory
2. Log files are automatically saved in the `logs/` directory
3. Attack experiment scripts support both Chinese and English versions
4. All scripts have been set with execute permissions
