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

### 5. Command
```bash
  ./scripts/demo/tmux_three_machine_demo.sh open

  ./scripts/demo/tmux_sros2_defense_demo.sh

  ./scripts/demo/tmux_sros2_dos_demo.sh

  ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 MITM_TAMPER_ENABLE=1 ./scripts/demo/tmux_gazebo_mitm_demo.sh
  
  ATTACK_DELAY_MS=500 ATTACK_LOSS_PERCENT=5 ATTACK_DURATION=45 ./scripts/demo/tmux_sros2_gazebo_mitm_demo.sh
```

## 📚 Documentation

- **Environment Setup**: `docs/env.md` - Complete Ubuntu + ROS2 + TurtleBot3 environment setup guide
- **Attack Experiments**: `attack_experiments/README_CN.md` - Chinese version attack experiment documentation
- **Attack Experiments**: `attack_experiments/README_EN.md` - English version attack experiment documentation
- **Experiment Index**: `experiments/README.md` - Presentation/report-oriented experiment folders and evidence checklist
- **Presentation Guide**: `docs/presentation/PRESENTATION_AND_REPORT_GUIDE.md` - Suggested PRE structure, result table, and report writing guidance
- **PRE Q&A Cheatsheet**: `docs/presentation/SECURITY_PRE_QA_CHEATSHEET.md` - Common defense questions for injection, SROS2, DoS, and MITM experiments
- **Final Report Draft**: `docs/FINAL_REPORT_DRAFT.md` - Final report draft based on the mid-progress report and migrated experiments
- **PPT Build Plan**: `docs/presentation/PPT_BUILD_PLAN.md` - Slide-by-slide presentation construction plan
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
