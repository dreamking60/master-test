# Using Empty World for Better Visualization

## Why Empty World?

Using an empty world (flat plane) in Gazebo provides several advantages for observing attack experiments:

1. **Clear Movement Patterns**: Robot movement is more visible on a flat, empty surface
2. **Better Trajectory Observation**: You can see the exact path the robot takes
3. **Easier Pattern Recognition**: Square, circle, and other patterns are more obvious
4. **Attack Effects More Visible**: Changes in behavior during attacks are easier to spot

## How to Use

### Start Gazebo with Empty World

```bash
# From project root
./scripts/setup/run_empty_world.sh
```

This launches Gazebo with:
- Empty flat plane (no obstacles)
- TurtleBot3 burger model
- Same ROS2 topics and functionality as normal world

### Run Attack Experiment

After Gazebo with empty world is running:

```bash
cd attack_experiments
./quick_test.sh
```

## What You'll See

### Normal Controller (Square Pattern)
- Robot moves in a clear square pattern
- Forward → Turn → Forward → Turn
- Easy to see the pattern on the flat surface

### During Attack
- Robot behavior changes are immediately visible
- If attack succeeds: Robot stops following square, goes straight
- Movement trajectory is clear on the empty plane

### Visual Indicators

**Before Attack**:
- Robot follows predictable square pattern
- Clear turning points visible

**During Attack**:
- Robot deviates from pattern
- Goes straight (override attack) or spins (spin attack)
- Trajectory clearly shows the hijack

**After Attack**:
- Robot returns to normal pattern (if normal controller still running)

## Comparison: Empty World vs Normal World

| Feature | Empty World | Normal World (turtlebot3_world) |
|---------|-------------|----------------------------------|
| Obstacles | None | Blue hexagonal obstacles |
| Visibility | Excellent | Good (but obstacles can block view) |
| Pattern Clarity | Very clear | Clear but may be obscured |
| Attack Observation | Easy to see | May be harder to observe |
| Use Case | Experiments, testing | Navigation, obstacle avoidance |

## Tips for Best Observation

1. **Camera Angle**: Adjust Gazebo camera to top-down view for best pattern visibility
2. **Grid**: The empty world has a grid, making it easy to measure movement
3. **Zoom**: Zoom out to see the full movement pattern
4. **Record**: Consider recording Gazebo window to analyze later

## Switching Between Worlds

You can switch between empty and normal worlds:

```bash
# Stop current Gazebo (Ctrl+C in the terminal running it)

# Start empty world
./scripts/setup/run_empty_world.sh

# Or start normal world
./scripts/setup/run.sh
```

Both provide the same ROS2 functionality - only the visual environment differs.

