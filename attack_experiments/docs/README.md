# /cmd_vel Injection Attack Experiments

This directory contains experiments demonstrating injection attacks on ROS2 robot control topics.

## 📁 Directory Structure

```
attack_experiments/
├── README.md                 # This file (main index)
├── README_EN.md             # English documentation
├── scripts/
│   ├── injection_attack.py  # Main attack script
│   └── deploy_to_real_robot.sh  # Real robot deployment guide
```

## 🚀 Quick Start

### Local Attack (Simulation)

1. Start the robot simulation:
```bash
cd ../..
./scripts/setup/run_empty_world.sh
```

2. In another terminal, run the attack:
```bash
cd attack_experiments/scripts
python3 injection_attack.py --attack-type turn_left --frequency 50 --duration 15
```

### Network Attack (Real Robot)

1. Run the deployment guide:
```bash
./deploy_to_real_robot.sh
```

2. Follow the interactive prompts to configure and attack.

## 📚 Documentation

- **English**: See [README_EN.md](README_EN.md)
- **INJECTION_EXPERIMENT.md** - How to run the injection experiment
- **LOGIC_AND_SUCCESS_CRITERIA.md** - How the scripts work
- **EXPERIMENT_RESULTS.md** - How to analyze results

## ⚠️ Warning

**This is for educational and security research purposes only.**
- Always obtain proper authorization before testing on real robots
- Use isolated network environments
- Backup important data before experiments

## 🔧 Requirements

- ROS2 (Humble or Jazzy)
- Python 3.8+
- rclpy package
- Network access (for network attacks)

