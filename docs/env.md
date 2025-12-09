# Final Project Practical Guide: Environment Setup and Basic Testing

This document is customized for **Mac (Apple Silicon M1/M2/M3)** environments, with the goal of completing the **TurtleBot Security Research Final Project**.

We will use the **UTM Virtual Machine + Ubuntu ARM64 + ROS 2** approach. This is currently the most stable and least problematic way to run robot simulations on Mac.

**Supported Ubuntu Versions:**
- **Ubuntu 22.04 (Jammy Jellyfish)** → Use **ROS 2 Humble**
- **Ubuntu 24.04 (Noble Numbat)** → Use **ROS 2 Jazzy**

> 💡 **Recommendation**: Ubuntu 22.04 + ROS 2 Humble is more stable with more mature packages. If you already have Ubuntu 24.04 installed, you can also use ROS 2 Jazzy.

-----

# 📘 Final Project Practical Guide: Environment Setup and Basic Testing

**Goal**: Build a virtual environment from scratch that can run TurtleBot3 simulations and verify that it can be controlled (preparing for subsequent attack experiments).
**Target Devices**: Mac (M1/M2/M3 chips)
**Estimated Total Time**: 1 - 2 hours

-----

## Phase One: Virtual Machine Setup (The Foundation)

We need a "fake" Linux computer to run ROS.

### Step 1: Download and Install UTM

UTM is the best open-source virtualization software for Mac (based on QEMU).

1. Go to [UTM official website](https://mac.getutm.app/) to download and install (free version is sufficient).

### Step 2: Download Ubuntu ARM64 Image

**Option A: Ubuntu 22.04 LTS (Jammy Jellyfish) - Recommended**
1. Go to Ubuntu official website to download [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://ubuntu.com/download/server/arm).
   * **Note**: Although it says "Server" here, it's the ARM64 architecture base. Or you can directly search and download **Ubuntu 22.04 Desktop ARM64** (if there's a desktop ISO, it's better, saving the trouble of configuring the interface).
   * *Recommended link*: Find Daily Build or directly use the Ubuntu 22.04 pre-configured version in UTM Gallery (easiest).

**Option B: Ubuntu 24.04 LTS (Noble Numbat)**
1. Go to Ubuntu official website to download [Ubuntu 24.04 LTS (Noble Numbat)](https://ubuntu.com/download/server/arm).
   * Also choose the ARM64 architecture version.
   * If using Ubuntu 24.04, you will install **ROS 2 Jazzy** (instead of Humble) later.

### Step 3: Configure Virtual Machine

1. Open UTM -> **Create a New Virtual Machine**.
2. Select **Virtualize** -> **Linux**.
3. Click **Browse** to select the downloaded `.iso` image file.
4. **Hardware Configuration (Critical)**:
   * **Memory**: Recommended **4096 MB (4GB)** or more.
   * **CPU Cores**: Recommended **4 Cores**.
5. **Shared Directory (Optional)**: You can set up a Mac folder to share with the virtual machine for convenient transfer of experiment report screenshots later.
6. **Start and Install**:
   * Start the virtual machine and follow the on-screen prompts to install Ubuntu.
   * *Tip*: If the graphical interface is very laggy during installation, or the screen goes black after installation, go to UTM settings -> Display -> uncheck **3D Acceleration** (or select `virtio-ramfb`).

### Step 3.5: Configure SSH Connection (Recommended)

**Why use SSH?** Connecting from Mac host to the virtual machine via SSH allows:
- More convenient copy-paste of commands
- Use Mac terminal tools (iTerm2, Terminal.app)
- Avoid possible performance issues with the virtual machine graphical interface
- Can open multiple terminal sessions simultaneously

**Configure SSH Server in Virtual Machine**:

1. Open Ubuntu terminal and install SSH server:
```bash
sudo apt update
sudo apt install openssh-server -y
```

2. Start SSH service and enable auto-start on boot:
```bash
sudo systemctl start ssh
sudo systemctl enable ssh
```

3. Check the virtual machine's IP address:
```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
```
   Or use:
```bash
hostname -I
```
   Note down the displayed IP address (usually `192.168.x.x` or `10.0.x.x`)

4. **Configure Network in UTM** (if needed):
   - Ensure the virtual machine's network mode is set to **Shared Network (NAT)** or **Bridged Network**
   - If using NAT, UTM will automatically assign IP
   - If using Bridged, the virtual machine will get an IP in the same network segment as Mac

**Connect from Mac Host**:

1. Open Mac terminal (Terminal.app or iTerm2)

2. Use SSH to connect (replace `<IP_address>` with the IP seen in step 3):
```bash
ssh <username>@<IP_address>
```
   For example: `ssh ubuntu@192.168.64.3`

3. First connection will prompt to confirm host key, type `yes`

4. Enter Ubuntu user password

**After successful connection**, you can directly operate the virtual machine in Mac terminal! All subsequent configuration commands can be executed in this SSH session.

**Tips**:
- If IP address changes frequently, you can set a static IP in the virtual machine, or use UTM's port forwarding feature
- You can use SSH key authentication to avoid entering password each time (optional)

-----

## Phase Two: ROS 2 Environment Deployment (The Software)

After the system is installed, **in the virtual machine terminal or SSH session**, execute the following commands line by line.

### Step 4: Set Locale

ROS 2 has strict locale requirements and must support UTF-8.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 5: Add ROS 2 Software Repository

Tell Ubuntu where to download ROS.

```bash
# Enable universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Download GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository address
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 6: Install ROS 2

This step takes some time, depending on your network speed.

**First confirm your Ubuntu version:**
```bash
lsb_release -cs
```

**Choose the corresponding ROS 2 version based on Ubuntu version:**

#### If Ubuntu 22.04 (Jammy) → Install ROS 2 Humble

```bash
sudo apt update
sudo apt upgrade
# Install desktop full version (includes Gazebo simulator)
sudo apt install ros-humble-desktop
```

#### If Ubuntu 24.04 (Noble) → Install ROS 2 Jazzy

```bash
sudo apt update
sudo apt upgrade
# Install desktop full version (includes Gazebo simulator)
sudo apt install ros-jazzy-desktop
```

> ⚠️ **Important**: Ensure ROS version matches Ubuntu version! Ubuntu 22.04 uses Humble, Ubuntu 24.04 uses Jazzy.

**⚠️ If installation fails, troubleshoot according to the following steps:**

#### Troubleshooting 1: Check if Software Repository is Correctly Added

First confirm if Step 5 was successfully executed:

```bash
# Check if ROS 2 software repository exists
cat /etc/apt/sources.list.d/ros2.list

# Should see output like:
# deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main
```

If the file doesn't exist or content is wrong, re-execute Step 5.

#### Troubleshooting 2: Check GPG Key

```bash
# Check if key file exists
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# If it doesn't exist, re-download:
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

#### Troubleshooting 3: Check System Architecture

UTM virtual machine should use ARM64 architecture (because Mac M1/M2/M3 are ARM chips):

```bash
# Check system architecture
dpkg --print-architecture
# Should output: arm64

# Check Ubuntu version
lsb_release -cs
# Should output: jammy (Ubuntu 22.04)
```

#### Troubleshooting 4: Update Package List and View Specific Errors

```bash
# Clean and update
sudo apt clean
sudo apt update

# View detailed error information
sudo apt install ros-humble-desktop 2>&1 | tee install_error.log
```

**Common Errors and Solutions:**

1. **Error: `E: Unable to locate package ros-humble-desktop` or `E: Unable to locate package ros-jazzy-desktop`**
   - **Cause 1**: Software repository not correctly added, or architecture mismatch
   - **Solution**: Re-execute Step 5, ensure architecture is `arm64`
   - **Cause 2**: ROS version doesn't match Ubuntu version
   - **Solution**:
     - Ubuntu 22.04 (Jammy) → Use `ros-humble-desktop`
     - Ubuntu 24.04 (Noble) → Use `ros-jazzy-desktop`
   - **Check method**:
     ```bash
     # Check Ubuntu version
     lsb_release -cs
     # If "noble", should install ros-jazzy-desktop
     # If "jammy", should install ros-humble-desktop
     ```

2. **Error: `GPG error: ... NO_PUBKEY ...`**
   - **Cause**: GPG key issue
   - **Solution**:
   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   sudo apt update
   ```

3. **Error: `404 Not Found` or network connection issue**
   - **Cause**: Network issue or mirror source issue
   - **Solution**: Check network connection, or try different mirror sources

4. **Error: Dependency conflict or package corruption**
   - **Solution**:
   ```bash
   sudo apt --fix-broken install
   sudo apt update
   sudo apt upgrade
   ```

#### Troubleshooting 5: If Above Doesn't Work, Try Step-by-Step Installation

If full version installation fails, try installing base version:

**ROS 2 Humble (Ubuntu 22.04):**
```bash
# Install base version first
sudo apt install ros-humble-desktop-base

# Then install required components separately
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-rviz2
```

**ROS 2 Jazzy (Ubuntu 24.04):**
```bash
# Install base version first
sudo apt install ros-jazzy-desktop-base

# Then install required components separately
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-rviz2
```

#### Troubleshooting 6: Verify Software Repository Configuration (Complete Checklist)

Run the following commands to ensure all configurations are correct:

```bash
# 1. Check architecture
echo "Architecture: $(dpkg --print-architecture)"

# 2. Check Ubuntu version
echo "Ubuntu version: $(lsb_release -cs)"

# 3. Check software repository file
echo "Software repository configuration:"
cat /etc/apt/sources.list.d/ros2.list

# 4. Check GPG key
echo "GPG key:"
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# 5. Test software repository connection
sudo apt update 2>&1 | grep -i "ros\|error\|failed"
```

If all checks pass but installation still fails, please send me the specific error information and I'll help you further diagnose.

### Step 7: Environment Initialization

Add ROS commands to your startup script so you can use them directly every time you open a terminal.

**Choose the corresponding command based on your installed ROS version:**

#### If ROS 2 Humble (Ubuntu 22.04)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### If ROS 2 Jazzy (Ubuntu 24.04)
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

-----

## Phase Three: Deploy TurtleBot3 Simulation (The Target)

We need to install your attack target (TurtleBot) into the system.

### Step 8: Install TurtleBot3 Related Packages

**Choose the corresponding command based on your installed ROS version:**

#### ROS 2 Humble (Ubuntu 22.04)
```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
```

#### ROS 2 Jazzy (Ubuntu 24.04)
```bash
sudo apt install ros-jazzy-gazebo-*
sudo apt install ros-jazzy-cartographer
sudo apt install ros-jazzy-cartographer-ros
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-turtlebot3*
```

### Step 9: Set Robot Model

TurtleBot has several models, we'll use the classic `burger` (hamburger style).

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

-----

## Phase Four: Verification and First Run (The Test)

This step determines whether you can start the core part of the Final Project.

### Step 10: Launch Simulation World

In terminal, type:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

  * **Expected Result**: A Gazebo window will pop up with a world containing blue hexagonal obstacles, and a small car (TurtleBot) parked in the middle of the floor.
  * *Mac-specific Troubleshooting*: If Gazebo opens with a black screen or crashes, go back to UTM settings -> Display -> uncheck "3D Acceleration" (3D acceleration), then restart the virtual machine.

### Step 11: Try Control (Simulating Attacker's First Step)

Keep the Step 10 window open, **open a new terminal window**, and type:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

  * **Operation**: Follow on-screen prompts, press `w` (forward), `a` (turn left), `x` (backward), `d` (turn right), `s` (stop) on the keyboard.
  * **Observation**: Does the robot in Gazebo move?
  * **Significance**: If you can control it to move, it means the `/cmd_vel` control interface is working.
      * Your **Attack Project** is to write a script that replaces this keyboard program, quietly sending `w` (forward) or `s` (stop) commands in the background to make the robot "lose control".

#### Step 11 Troubleshooting: Keyboard Control Failure

If `teleop_keyboard` cannot work properly, troubleshoot step by step:

**Troubleshooting 1: Check if turtlebot3_teleop package is installed**

```bash
# Check if package exists
ros2 pkg list | grep turtlebot3_teleop

# If not found, reinstall (choose based on your ROS version)
# ROS 2 Humble (Ubuntu 22.04):
sudo apt install ros-humble-turtlebot3-teleop

# ROS 2 Jazzy (Ubuntu 24.04):
sudo apt install ros-jazzy-turtlebot3-teleop
```

**Troubleshooting 2: Check if ROS2 Environment is Correctly Set**

In a new terminal window, ensure ROS2 environment is loaded:

```bash
# Check ROS_DISTRO environment variable
echo $ROS_DISTRO
# Should output: humble or jazzy

# If no output, manually load environment (choose based on your ROS version)
# ROS 2 Humble:
source /opt/ros/humble/setup.bash

# ROS 2 Jazzy:
source /opt/ros/jazzy/setup.bash

# Check if TURTLEBOT3_MODEL is set
echo $TURTLEBOT3_MODEL
# Should output: burger

# If not, set it:
export TURTLEBOT3_MODEL=burger
```

**Troubleshooting 3: Check if Gazebo Simulation is Running**

Ensure the Step 10 Gazebo window is still running and you can see the robot. Then check topics:

```bash
# Check if /cmd_vel topic exists
ros2 topic list | grep cmd_vel

# Should see: /cmd_vel

# Check topic type
ros2 topic info /cmd_vel

# Should show type as: geometry_msgs/msg/TwistStamped
```

**Troubleshooting 4: Keyboard Input Issues in SSH Session (Most Common)**

If you're connecting to the virtual machine via SSH, keyboard input may not work properly. This is because `teleop_keyboard` requires raw terminal mode to read individual keystrokes.

**Solution A: Run in Virtual Machine Local Terminal (Recommended)**

Directly run `teleop_keyboard` in the terminal within the UTM virtual machine window, not via SSH.

**Solution B: Use SSH but Configure Correct Terminal Mode**

```bash
# Ensure your terminal supports raw mode
# In Mac terminal, try:
stty raw -echo

# Then run teleop_keyboard
ros2 run turtlebot3_teleop teleop_keyboard

# After use, restore terminal settings:
stty sane
```

**Solution C: Use Alternative Control Method (Bypass Keyboard Issue)**

If keyboard input really doesn't work, you can directly publish control commands to test:

```bash
# In new terminal, directly publish forward command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# Publish stop command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# Publish rotation command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

If these commands can make the robot move, it means the `/cmd_vel` interface is normal, the problem is just keyboard input.

**Troubleshooting 5: Check if Node is Running Properly**

```bash
# Check if teleop_keyboard node is running
ros2 node list | grep teleop

# Check node information
ros2 node info /teleop_keyboard
```

**Troubleshooting 6: View Detailed Error Information**

When running teleop_keyboard, check if there's error output:

```bash
# Run and view detailed output
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --log-level debug
```

**Troubleshooting 7: Verify Topic Connection**

While running `teleop_keyboard`, check in another terminal if messages are being published:

```bash
# Listen to /cmd_vel topic to see if there are messages
ros2 topic echo /cmd_vel

# When you press keyboard, you should see message output
```

**Quick Diagnosis Script**

Use the diagnosis script in the project (if available):

```bash
# Run diagnosis script
./scripts/testing/diagnose_teleop.sh
```

Or manually run diagnosis commands:

```bash
echo "ROS_DISTRO: $ROS_DISTRO"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
ros2 pkg list | grep turtlebot3_teleop
ros2 topic list | grep cmd_vel
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Alternative Control Solution: Use Python Script**

If keyboard input really doesn't work, you can use the `test_control.py` script in the project to control the robot:

```bash
# Ensure in ROS2 environment
source /opt/ros/humble/setup.bash  # or jazzy
export TURTLEBOT3_MODEL=burger

# Forward for 2 seconds
python3 scripts/testing/test_control.py forward 0.2 2.0

# Turn left for 1 second
python3 scripts/testing/test_control.py left 0.5 1.0

# Turn right for 1 second
python3 scripts/testing/test_control.py right 0.5 1.0

# Stop
python3 scripts/testing/test_control.py stop

# Spin attack (for testing)
python3 scripts/testing/test_control.py spin 3.0
```

This script can verify if the `/cmd_vel` interface works properly, even if keyboard input has issues.

**If Above Doesn't Work, Try Reinstalling**

```bash
# Completely reinstall turtlebot3 related packages (choose based on your ROS version)
# ROS 2 Humble:
sudo apt remove ros-humble-turtlebot3*
sudo apt autoremove
sudo apt install ros-humble-turtlebot3*

# ROS 2 Jazzy:
sudo apt remove ros-jazzy-turtlebot3*
sudo apt autoremove
sudo apt install ros-jazzy-turtlebot3*
```

-----

## 📋 ROS2 Logging System Documentation

ROS2 provides a complete logging system that can help you debug and monitor robot runtime status.

### ROS2 Log Storage Location

**Default Log Directory:**
```bash
# View default log directory
echo ~/.ros/log

# List all log files
ls -lh ~/.ros/log/
```

**Log File Naming Rules:**
- Log files are usually named by node name and timestamp
- Format similar to: `node_name_timestamp.log`

### Custom Log Directory

You can customize log storage location through environment variables:

```bash
# Method 1: Use ROS_LOG_DIR (highest priority)
export ROS_LOG_DIR=/path/to/your/logs
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Method 2: Use ROS_HOME (if ROS_LOG_DIR is not set)
export ROS_HOME=/path/to/ros/home
# Logs will be stored in $ROS_HOME/log directory

# Method 3: Temporarily set in launch command
ROS_LOG_DIR=./logs ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Methods to View Logs

**Method 1: Use rqt_console (GUI, Recommended)**

```bash
# Install rqt_console (if not installed)
sudo apt install ros-humble-rqt-console  # or ros-jazzy-rqt-console

# Launch log viewer
ros2 run rqt_console rqt_console
```

`rqt_console` features:
- Real-time display of all node log messages
- Filter by log level (DEBUG, INFO, WARN, ERROR, FATAL)
- Save logs to file
- Load saved log files
- Search and filter functions

**Method 2: Directly View Log Files**

```bash
# View latest log files
ls -lt ~/.ros/log/ | head -5

# View specific log file content
tail -f ~/.ros/log/latest_log_file_name.log

# Search for errors
grep -i error ~/.ros/log/*.log

# Search for warnings
grep -i warn ~/.ros/log/*.log
```

**Method 3: View Real-time Logs in Terminal**

```bash
# Set log level and run node
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --log-level debug

# Or use info level (default)
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --log-level info
```

### Log Levels

ROS2 supports the following log levels (from low to high):
- **DEBUG**: Detailed debugging information
- **INFO**: General information (default level)
- **WARN**: Warning information
- **ERROR**: Error information
- **FATAL**: Fatal error

**Set Log Level:**
```bash
# Set in launch command
ros2 run package_name node_name --ros-args --log-level debug

# Or set in code (Python)
rclpy.logging.set_logger_level('node_name', rclpy.logging.LoggingSeverity.DEBUG)
```

### Useful Log-Related Commands

```bash
# View all running nodes
ros2 node list

# View detailed information of specific node (including log configuration)
ros2 node info /node_name

# View all topics
ros2 topic list

# Listen to topic messages (this is also a type of "log")
ros2 topic echo /cmd_vel

# View topic publish frequency
ros2 topic hz /cmd_vel

# View topic bandwidth usage
ros2 topic bw /cmd_vel
```

### Combine Logs with Launch Script

If you want to automatically save logs when launching Gazebo:

```bash
# Modify run.sh, add log directory setting
#!/bin/bash
export ROS_LOG_DIR=./logs
mkdir -p logs
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py > test.log 2>&1
```

Or save to file and display in terminal simultaneously:

```bash
#!/bin/bash
export ROS_LOG_DIR=./logs
mkdir -p logs
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 2>&1 | tee test.log
```

### Log Analysis Tips

```bash
# Count error numbers
grep -c "ERROR" ~/.ros/log/*.log

# View recent errors
grep "ERROR" ~/.ros/log/*.log | tail -20

# View logs sorted by time
find ~/.ros/log -name "*.log" -exec ls -lt {} + | head -10

# View logs of specific node
grep "node_name" ~/.ros/log/*.log
```

### Clean Old Logs

```bash
# Delete all logs (use with caution)
rm -rf ~/.ros/log/*

# Only delete logs older than 7 days
find ~/.ros/log -name "*.log" -mtime +7 -delete

# View log directory size
du -sh ~/.ros/log/
```

-----

## ✅ Next Steps Checkpoint

After you complete **Step 11** and successfully make the robot move, please let me know.

Next, I will teach you how to perform the first attack experiment:
**Write a Python script that masquerades as a legitimate node and sends "death spin" commands to TurtleBot.**

**Now, please go set up UTM and Ubuntu first!**
