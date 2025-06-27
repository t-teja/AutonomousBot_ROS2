# RP2040 Motor Control Protocol - DEFINITIVE GUIDE

**🎯 CRITICAL SUCCESS INFORMATION: This document contains the EXACT working solution for JetRacer RP2040 motor control.**

## ✅ CONFIRMED WORKING SOLUTION

### Hardware Configuration
- **Jetson Orin Nano** with JetPack 6.2 and ROS2 Humble
- **Waveshare JetRacer** with RP2040 secondary controller
- **USB Connection**: RP2040 appears as `/dev/ttyACM1` (NOT ttyACM0)
- **Encoder LEDs**: Must be ON to indicate hardware is functional
- **Power**: Robot must be powered on with power LED lit

### Serial Communication Details
- **Port**: `/dev/ttyACM1` (confirmed working)
- **Baudrate**: 115200
- **Protocol**: Binary format (NOT JSON)
- **Packet Format**: `0xAA 0x55 [length] [data...] [checksum]`

### ✅ WORKING ROS2 NODE
**Use the backup node**: `jetracer_project_backup-27-06-25/Jetracer/scripts/jetracer_servo_node_with_encoders.py`

**Command to run**:
```bash
cd /home/orin/Documents/jetros
python3 jetracer_project_backup-27-06-25/Jetracer/scripts/jetracer_servo_node_with_encoders.py --ros-args -p serial_port:=/dev/ttyACM1
```

### ✅ CONFIRMED WORKING COMMANDS

#### Forward Movement
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Steering Only
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

#### Combined Movement + Steering
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### ✅ ENCODER DATA VERIFICATION
**Correct encoder byte positions** (confirmed working):
- **Left Wheel Velocity**: bytes 34-35 (big-endian 16-bit signed)
- **Right Wheel Velocity**: bytes 36-37 (big-endian 16-bit signed)
- **Left Wheel Setpoint**: bytes 38-39 (big-endian 16-bit signed)
- **Right Wheel Setpoint**: bytes 40-41 (big-endian 16-bit signed)

**Expected behavior**:
- Values = 0 when robot is stationary
- Values change when wheels are manually rotated
- Values change to ~56 when motors are commanded

## 🚫 WHAT DOESN'T WORK

### ❌ Current jetracer_base Node
The current ROS2 package expects JSON communication but RP2040 uses binary protocol.
**Error**: `'utf-8' codec can't decode byte 0xaa in position 0: invalid start byte`

### ❌ Direct Serial Commands
Direct motor commands via serial do NOT work. Must use ROS2 topics.

### ❌ Wrong Serial Port
Using `/dev/ttyACM0` will fail. Must use `/dev/ttyACM1`.

## 📋 TROUBLESHOOTING CHECKLIST

### 1. Hardware Check
```bash
ls -la /dev/ttyACM*  # Should show ttyACM1
```
- Encoder LEDs should be ON
- Power LED should be ON
- Robot should be on flat surface

### 2. Serial Permissions
```bash
groups  # Should include 'dialout'
sudo usermod -a -G dialout $USER  # If not in dialout group
```

### 3. ROS2 Environment
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash
```

### 4. Node Status Check
```bash
ros2 topic list  # Should show /cmd_vel
ros2 node list   # Should show jetracer_servo_node_with_encoders
```

### 5. Encoder Verification
Monitor encoder values - they should be 0 when stationary and change when wheels move.

## 🔧 STEP-BY-STEP STARTUP PROCEDURE

### Step 1: Hardware Verification
1. Power on robot (power LED should be lit)
2. Check encoder LEDs are ON
3. Verify USB connection: `ls -la /dev/ttyACM*`

### Step 2: Start ROS2 Node
```bash
cd /home/orin/Documents/jetros
python3 jetracer_project_backup-27-06-25/Jetracer/scripts/jetracer_servo_node_with_encoders.py --ros-args -p serial_port:=/dev/ttyACM1
```

**Expected output**:
```
[INFO] ✅ Connected to RP2040 controller
[INFO] Jetracer Servo Node with Encoders started
[INFO] Encoder data - Left vel: 0, Right vel: 0, Left set: 0, Right set: 0
```

### Step 3: Test Movement
```bash
# In new terminal:
cd /home/orin/Documents/jetros
source install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Expected behavior**:
- Robot moves forward
- Encoder values change from 0 to ~56 and back to 0
- Log shows: `Binary cmd_vel: linear_x=0.300m/s, angular_z=0.000rad`

## 📊 OFFICIAL WAVESHARE SPECIFICATIONS

According to [Waveshare JetRacer ROS AI Kit Tutorial V](https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit_Tutorial_V:_Robot_Movement_Control):

- **linear.x**: Linear speed (-1.2 to 1.2 m/s)
- **angular.z**: Steering angle (-0.6 to 0.6 radians) - NOT angular velocity!
- **Topic**: `/cmd_vel` with `geometry_msgs/msg/Twist` type

## 🎯 CRITICAL SUCCESS FACTORS

1. **Use the backup node** - NOT the current jetracer_base package
2. **Use /dev/ttyACM1** - NOT ttyACM0
3. **Binary protocol** - NOT JSON
4. **ROS2 topics** - NOT direct serial commands
5. **Encoder LEDs ON** - Hardware must be functional

## 📁 FILE LOCATIONS

- **Working node**: `jetracer_project_backup-27-06-25/Jetracer/scripts/jetracer_servo_node_with_encoders.py`
- **Workspace**: `/home/orin/Documents/jetros/Autonomous_robot`
- **Serial device**: `/dev/ttyACM1`

## 🔬 TECHNICAL DETAILS - BINARY PROTOCOL

### RP2040 Communication Format
The RP2040 uses Waveshare's binary packet format:

**Packet Structure**:
```
[0xAA] [0x55] [LENGTH] [DATA...] [CHECKSUM]
```

**Example Motor Command Packet**:
```
AA 55 07 01 2C 01 2C 01 F9
│  │  │  │  │     │     │
│  │  │  │  │     │     └─ Checksum (XOR of all bytes)
│  │  │  │  │     └─ Right motor value (300 = 0x012C)
│  │  │  │  └─ Left motor value (300 = 0x012C)
│  │  │  └─ Command type (0x01 = motor command)
│  │  └─ Packet length (7 bytes)
│  └─ Header byte 2
└─ Header byte 1
```

### Encoder Data Format
**Incoming packet from RP2040** (45 bytes total):
- Bytes 0-1: Header (0xAA 0x55)
- Bytes 2: Length (45)
- Bytes 34-35: Left wheel velocity (big-endian signed 16-bit)
- Bytes 36-37: Right wheel velocity (big-endian signed 16-bit)
- Bytes 38-39: Left wheel setpoint (big-endian signed 16-bit)
- Bytes 40-41: Right wheel setpoint (big-endian signed 16-bit)

### ROS2 to RP2040 Command Translation
The working node (`jetracer_servo_node_with_encoders.py`) translates:

**ROS2 Twist Message**:
```python
linear.x = 0.5    # m/s forward speed
angular.z = 0.3   # radians steering angle
```

**To RP2040 Binary Commands**:
1. **Motor speeds**: Calculated from linear.x and angular.z
2. **Servo angle**: Direct mapping from angular.z
3. **Binary packet**: Formatted as Waveshare protocol

### Key Code Locations in Working Node
- **Line 221-225**: Encoder data parsing (CORRECT byte positions)
- **Line 150-180**: cmd_vel callback and motor calculation
- **Line 200-220**: Binary packet construction
- **Line 31**: Serial port parameter (change to /dev/ttyACM1)

## 🚨 COMMON MISTAKES TO AVOID

1. **Using wrong node**: Current jetracer_base expects JSON, use backup node
2. **Wrong serial port**: ttyACM0 vs ttyACM1 - use ttyACM1
3. **Direct serial commands**: Don't work - use ROS2 topics only
4. **JSON commands**: RP2040 firmware uses binary protocol
5. **Wrong encoder bytes**: Use bytes 34-41, not other positions

## 📞 EMERGENCY RECOVERY

If robot stops responding:
1. Check power and encoder LEDs
2. Restart the ROS2 node
3. Verify /dev/ttyACM1 exists
4. Check USB cable connection
5. Power cycle the robot

---

**⚠️ IMPORTANT**: This document contains the EXACT working solution verified on 2025-06-27. If this exact procedure doesn't work, check hardware connections and power first. The software solution is confirmed working as documented above.
