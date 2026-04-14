# TurtleBot3 Security Research Project

ROS2 TurtleBot3 security research project, including environment setup, basic testing, and attack experiments.

## 📁 Project Structure

```
master-test/
├── README.md                 # This file
├── attack_experiments/       # Attack experiment related
│   ├── README.md            # Attack experiment index
│   ├── README_CN.md         # Chinese documentation
│   ├── README_EN.md         # English documentation
│   ├── attack_cmd_vel_CN.py # Chinese version attack script
│   ├── attack_cmd_vel_EN.py # English version attack script
│   └── deploy_to_real_robot.sh # Real robot deployment script
├── scripts/                  # Scripts directory
│   ├── setup/               # Setup and run scripts
│   │   ├── run.sh           # Launch Gazebo simulation
│   │   └── keyboard.sh      # Keyboard control script
│   └── testing/              # Testing and diagnostic scripts
│       ├── check_status.sh  # System status check
│       ├── diagnose_teleop.sh # Keyboard control diagnosis
│       ├── test_control.py   # Control test script
│       ├── test_movement.sh  # Movement test script
│       └── view_logs.sh      # Log viewer tool
├── docs/                     # Documentation directory
│   ├── env.md               # Environment setup guide
│   └── cmd_vel_injection_attack.md # Attack experiment documentation (old version)
└── logs/                     # Logs directory
    └── ...                   # ROS2 runtime logs
```

## 🚀 Quick Start

### 1. Environment Setup

Follow the steps in `docs/env.md` to set up the ROS2 environment.

### 2. Launch Simulation

```bash
./scripts/setup/run.sh
```

### 3. Test Control

```bash
# Method 1: Use keyboard control
./scripts/setup/keyboard.sh

# Method 2: Use Python script
python3 scripts/testing/test_control.py forward 0.2 2.0
```

### 4. Run Attack Experiments

```bash
cd attack_experiments
python3 attack_cmd_vel_EN.py --attack forward --speed 0.5 --duration 10
```

## 📚 Documentation

- **Environment Setup**: `docs/env.md` - Complete Ubuntu + ROS2 + TurtleBot3 environment setup guide
- **Attack Experiments**: `attack_experiments/README_CN.md` - Chinese version attack experiment documentation
- **Attack Experiments**: `attack_experiments/README_EN.md` - English version attack experiment documentation
- **WSL + Docker Hybrid Setup**: `deployment/wsl_docker/README.md` - Gazebo on WSL host, controller/attacker in containers
- **Short-term Plan**: `docs/SHORT_TERM_PLAN.md` - 1-month detailed plan (最近1个月计划)
- **Long-term Plan**: `docs/LONG_TERM_PLAN.md` - Long-term project roadmap (长期项目路线图)

## 🛠️ Tool Scripts

### Setup Scripts (`scripts/setup/`)
- `run.sh` - Launch Gazebo simulation environment
- `keyboard.sh` - Launch keyboard control

### Testing Scripts (`scripts/testing/`)
- `check_status.sh` - Check ROS2 system status
- `diagnose_teleop.sh` - Diagnose keyboard control issues
- `test_control.py` - Python control test script
- `test_movement.sh` - Movement test script
- `view_logs.sh` - Log viewer tool

## ⚠️ Important Notes

1. **Attack experiments are for educational purposes only** - Ensure experiments are conducted in isolated environments
2. **Real robot testing** requires explicit authorization before testing
3. It is recommended to run experiments in virtual machines or isolated networks

## 📝 License

This project is for educational and security research purposes only.
