# Gazebo View Fix - Can't See Robot

## Problem

When Gazebo opens, you see something floating in the sky and can't see the robot on the ground.

## Quick Fix

### Method 1: Reset Camera View (Easiest)

1. **In Gazebo window**:
   - Press **`F`** key to focus on the robot
   - Or click on "burger" in the model list (left panel), then press **`F`**

2. **Manual camera control**:
   - **Right-click + drag**: Rotate view
   - **Middle-click + drag**: Pan view  
   - **Scroll wheel**: Zoom in/out
   - **Ctrl + drag**: Move camera

### Method 2: Check if Robot Spawned

Open a new terminal and check:

```bash
# Check if robot topics exist
ros2 topic list | grep cmd_vel

# Check if robot node exists
ros2 node list | grep burger

# Check robot position
ros2 topic echo /odom --once
```

If topics don't exist, the robot didn't spawn correctly.

### Method 3: Restart Gazebo

```bash
# Kill existing Gazebo
pkill -f gazebo
pkill -f gz

# Wait a moment
sleep 2

# Restart
cd /home/stevenchen/master-test
./scripts/setup/run_empty_world.sh
```

## Common Issues

### Issue 1: Camera Looking at Sky

**Solution**: 
- Press **`F`** to focus on robot
- Or use mouse to rotate view down

### Issue 2: Robot Not Spawned

**Check**:
```bash
ros2 topic list
```

If `/cmd_vel` doesn't exist, robot didn't spawn.

**Fix**:
1. Make sure `TURTLEBOT3_MODEL=burger` is set
2. Check logs: `tail -f test.log`
3. Restart Gazebo

### Issue 3: Robot Spawned But Invisible

**Solution**:
- Press **`F`** to focus
- Zoom out (scroll wheel)
- Check if robot is at origin (0, 0, 0)

## Expected View

After fixing, you should see:
- Flat empty plane (gray/white ground)
- Small robot (TurtleBot3 burger) in the center
- Clear view from above or side angle

## Keyboard Shortcuts

- **`F`**: Focus on selected model
- **`Ctrl + F`**: Frame selected model
- **`Esc`**: Deselect
- **Right-click + drag**: Rotate camera
- **Middle-click + drag**: Pan camera
- **Scroll**: Zoom

