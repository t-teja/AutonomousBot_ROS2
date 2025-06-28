# ROS Topics Documentation - JetRacer Autonomous Robot

## Overview
This document provides a comprehensive overview of all ROS2 nodes, topics, services, and actions in the JetRacer autonomous robot project. It includes communication flows and system architecture.

**Last Updated**: 2025-06-28  
**Maintainer**: Teja  
**ROS2 Version**: Humble  

---

## 🤖 ROS2 Nodes

### 1. motor_control_node (motor_control_pkg)
**Purpose**: Motor and servo control interface with RP2040 secondary controller  
**Executable**: `motor_control_node`  
**Status**: ✅ ACTIVE AND WORKING  

#### Subscribed Topics:
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for robot movement

#### Published Topics:
- `/odom` (nav_msgs/Odometry) - Robot odometry data
- `/joint_states` (sensor_msgs/JointState) - Motor joint states
- `/battery_voltage` (std_msgs/Float32) - Battery voltage from RP2040
- `/robot_status` (std_msgs/String) - Robot system status

#### TF Frames:
- Publishes: `odom` → `base_link` transform

---

### 2. oled_display_node (oled_display_pkg)
**Purpose**: OLED display control for IP address and system information  
**Executable**: `oled_display_node`  
**Status**: ✅ ACTIVE AND WORKING  

#### Subscribed Topics:
- `/oled_display_message` (std_msgs/String) - Custom display messages

#### Published Topics:
- `/robot_ip_address` (std_msgs/String) - Current robot IP address

---

### 3. battery_monitor_node (battery_monitor_pkg)
**Purpose**: INA219 battery monitoring and power management
**Executable**: `battery_monitor_node`
**Status**: ✅ ACTIVE AND WORKING

#### Published Topics:
- `/battery_state` (sensor_msgs/BatteryState) - Standard ROS battery state
- `/jetracer/battery_state` (jetracer_msgs/BatteryState) - Custom battery state
- `/battery/voltage` (std_msgs/Float32) - Battery voltage (V)
- `/battery/current` (std_msgs/Float32) - Battery current (A)
- `/battery/power` (std_msgs/Float32) - Battery power (W)
- `/battery/percentage` (std_msgs/Float32) - Battery percentage (0-100)
- `/battery/status` (std_msgs/String) - Battery status text
- `/battery/low_battery_warning` (std_msgs/Bool) - Low battery alert
- `/battery/critical_battery_warning` (std_msgs/Bool) - Critical battery alert

---

### 4. rplidar_node (rplidar_ros package)
**Purpose**: RPLidar A1 laser scanner for SLAM and navigation
**Executable**: `rplidar_node`
**Status**: ✅ ACTIVE AND WORKING
**Hardware**: RPLidar A1M8 via Waveshare JetRacer board (/dev/ttyACM1)

#### Published Topics:
- `/scan` (sensor_msgs/LaserScan) - Laser scan data for SLAM/navigation

#### Hardware Specifications:
- **Model**: RPLidar A1M8
- **Firmware**: Version 1.29, Hardware Rev: 7
- **Range**: 0.15m to 12.0m
- **Scan Rate**: ~7.8 Hz (actual measured)
- **Sample Rate**: 8 KHz
- **Scan Mode**: Sensitivity

#### TF Frames:
- **Published Frame**: `laser`
- **Transform**: Static transform base_link → laser (when using full launch)

---

### 5. web_server_node (web_interface_pkg)
**Purpose**: HTTP server for modern web interface
**Executable**: `web_server_node.py`
**Status**: ✅ COMPLETE AND READY

#### Features:
- Modern responsive web interface optimized for mobile/tablet
- Real-time robot control with touch-friendly joystick
- Battery monitoring dashboard
- Lidar scan visualization
- System status monitoring
- Progressive Web App (PWA) support

#### HTTP Endpoints:
- `/` - Main robot control interface
- `/api/robot_info` - Robot information API
- `/api/system_status` - System status API

---

### 6. ros_bridge_node (web_interface_pkg)
**Purpose**: WebSocket bridge between ROS2 and web interface
**Executable**: `ros_bridge_node.py`
**Status**: ✅ COMPLETE AND READY

#### Subscribed Topics:
- `/battery_state` (sensor_msgs/BatteryState) - Battery monitoring
- `/jetracer/battery_state` (jetracer_msgs/BatteryState) - Custom battery data
- `/battery/voltage` (std_msgs/Float32) - Battery voltage
- `/battery/percentage` (std_msgs/Float32) - Battery percentage
- `/robot_status` (std_msgs/String) - Robot status
- `/robot_ip_address` (std_msgs/String) - Robot IP
- `/scan` (sensor_msgs/LaserScan) - Lidar scan data
- `/lidar/health_status` (std_msgs/Bool) - Lidar health
- `/odom` (nav_msgs/Odometry) - Robot odometry

#### Published Topics:
- `/cmd_vel` (geometry_msgs/Twist) - Robot movement commands
- `/oled_display_message` (std_msgs/String) - OLED display messages

#### WebSocket Features:
- Real-time data streaming at 10Hz
- Bidirectional communication
- Robot control commands
- Lidar/battery data visualization
- System monitoring

---

## 📨 Custom Message Types (jetracer_msgs)

### Messages (.msg)
1. **BatteryState.msg** - Comprehensive battery monitoring
2. **MotorCommand.msg** - Motor control commands
3. **MotorState.msg** - Motor status feedback
4. **ServoCommand.msg** - Steering servo commands
5. **ServoState.msg** - Servo status feedback
6. **SystemStatus.msg** - Overall system status
7. **WheelSpeeds.msg** - Wheel encoder data

### Services (.srv)
1. **Calibrate.srv** - System calibration service
2. **SetMotorPID.srv** - Motor PID parameter tuning

---

## 🔄 Communication Flow

### Robot Movement Control
```
External Command → /cmd_vel → motor_control_node → RP2040 → Motors/Servo
```

### Odometry Feedback
```
RP2040 Encoders → motor_control_node → /odom → Navigation Stack
```

### Battery Monitoring
```
INA219 Sensor → battery_monitor_node → Multiple /battery/* topics → System Monitoring
```

### Display System
```
IP Detection → oled_display_node → /robot_ip_address → System Info
Custom Messages → /oled_display_message → oled_display_node → OLED Display
```

---

## 🚀 Launch Files

### 1. motor_control.launch.py
**Package**: motor_control_pkg  
**Purpose**: Launches motor control system  
**Parameters**:
- `serial_port`: RP2040 serial port (default: /dev/ttyACM0)
- `baudrate`: Serial communication rate (default: 115200)
- `wheel_base`: Robot wheelbase in meters (default: 0.15)
- `max_speed`: Maximum speed in m/s (default: 1.0)
- `max_steering_angle`: Maximum steering angle in degrees (default: 30.0)

### 2. oled_display.launch.py
**Package**: oled_display_pkg  
**Purpose**: Launches OLED display system  
**Parameters**:
- `i2c_bus`: I2C bus number (default: 7)
- `i2c_address`: OLED I2C address (default: 0x3C)
- `update_rate`: Display update rate in Hz (default: 2.0)
- `ip_refresh_interval`: IP check interval in seconds (default: 30.0)

### 3. battery_monitor.launch.py
**Package**: battery_monitor_pkg
**Purpose**: Launches battery monitoring system
**Parameters**:
- `i2c_bus`: I2C bus number (default: 7)
- `i2c_address`: INA219 I2C address (default: 0x41)
- `publish_rate`: Data publish rate in Hz (default: 1.0)
- `nominal_voltage`: Battery nominal voltage (default: 11.1)
- `max_voltage`: Battery maximum voltage (default: 12.6)
- `min_voltage`: Battery minimum voltage (default: 9.0)
- `capacity`: Battery capacity in Ah (default: 7.8)

### 4. RPLidar Launch Files
**Package**: rplidar_ros
**Purpose**: Launches RPLidar A1 with various configurations

#### 4a. rplidar_a1_launch.py
**Purpose**: Basic RPLidar A1 launch
**Parameters**:
- `serial_port`: RPLidar serial port (default: /dev/ttyUSB0, use /dev/ttyACM1 for JetRacer)
- `frame_id`: Laser frame ID (default: laser)
- `inverted`: Invert scan data (default: false)
- `angle_compensate`: Enable angle compensation (default: true)
- `scan_mode`: Scan mode (default: Sensitivity)

#### 4b. view_rplidar_a1_launch.py
**Purpose**: RPLidar A1 with RViz visualization
**Parameters**: Same as rplidar_a1_launch.py
**Additional**: Launches RViz with pre-configured rplidar_ros.rviz file

#### 4c. waveshare_jetracer_rplidar_launch.py
**Purpose**: Waveshare JetRacer specific RPLidar launch
**Parameters**: Pre-configured for /dev/ttyACM1 and JetRacer setup

---

## 🔧 Hardware Interfaces

### I2C Bus 7 (External Safe Bus)
- **OLED Display**: Address 0x3C
- **INA219 Battery Monitor**: Address 0x41
- **GPIO Pins**: 3 (SDA), 5 (SCL)

### USB Serial Communication
- **RP2040 Controller**: /dev/ttyACM0 (115200 baud)
- **Protocol**: Binary format (not JSON)

### TF Tree Structure
```
odom
└── base_link
```

---

## ⚠️ Important Notes

### I2C Safety Warning
- **NEVER scan internal I2C buses** (especially bus 5 - CSI camera)
- **Only use Bus 7** for external peripherals
- **Scanning internal buses WILL CAUSE SYSTEM CRASHES**

### Communication Protocol
- **RP2040 Communication**: Binary protocol (NOT JSON)
- **Working Implementation**: Confirmed in backup folder
- **Encoder Hardware**: Functional (LEDs ON indicates proper operation)

### System Dependencies
- **ROS2 Humble** on Ubuntu with JetPack 6.2
- **Python 3.10** for all node implementations
- **Hardware**: Jetson Orin Nano + Waveshare JetRacer platform

---

## 📋 Future Expansion Nodes

### Planned Additions
1. **lidar_node** - RPLidar A1 integration
2. **camera_node** - IMX219 camera interface
3. **imu_node** - 9520 IMU integration
4. **slam_node** - SLAM mapping system
5. **navigation_node** - Path planning and navigation
6. **web_interface_node** - Tablet interface for goal selection
7. **aruco_docking_node** - Marker-based docking system

### Topic Expansion Plan
- `/scan` - Lidar data
- `/camera/image_raw` - Camera feed
- `/imu/data` - IMU measurements
- `/map` - SLAM generated map
- `/goal_pose` - Navigation goals
- `/cmd_vel_mux` - Velocity command multiplexer

---

## 📊 System Architecture Diagram

The following Mermaid diagram shows the complete ROS2 communication flow:

```mermaid
graph TB
    %% External Inputs
    EXT[External Commands] --> CMD[/cmd_vel<br/>geometry_msgs/Twist]
    CUSTOM[Custom Display Messages] --> DISP_MSG[/oled_display_message<br/>std_msgs/String]

    %% Main Nodes
    subgraph "ROS2 Nodes"
        MCN[motor_control_node<br/>motor_control_pkg]
        ODN[oled_display_node<br/>oled_display_pkg]
        BMN[battery_monitor_node<br/>battery_monitor_pkg]
    end

    %% Hardware Interfaces
    subgraph "Hardware Layer"
        RP2040[RP2040 Controller<br/>/dev/ttyACM0<br/>Binary Protocol]
        OLED[OLED Display<br/>I2C Bus 7, 0x3C]
        INA219[INA219 Sensor<br/>I2C Bus 7, 0x41]
        MOTORS[DC Motors + Encoders]
        SERVO[Steering Servo]
    end

    %% Topic Flows - Motor Control
    CMD --> MCN
    MCN --> ODOM[/odom<br/>nav_msgs/Odometry]
    MCN --> JOINTS[/joint_states<br/>sensor_msgs/JointState]
    MCN --> BAT_VOLT[/battery_voltage<br/>std_msgs/Float32]
    MCN --> STATUS[/robot_status<br/>std_msgs/String]
    MCN <--> RP2040
    RP2040 --> MOTORS
    RP2040 --> SERVO

    %% Topic Flows - OLED Display
    DISP_MSG --> ODN
    ODN --> IP_ADDR[/robot_ip_address<br/>std_msgs/String]
    ODN --> OLED

    %% Topic Flows - Battery Monitor
    BMN --> BAT_STATE[/battery_state<br/>sensor_msgs/BatteryState]
    BMN --> JET_BAT[/jetracer/battery_state<br/>jetracer_msgs/BatteryState]
    BMN --> BAT_V[/battery/voltage<br/>std_msgs/Float32]
    BMN --> BAT_C[/battery/current<br/>std_msgs/Float32]
    BMN --> BAT_P[/battery/power<br/>std_msgs/Float32]
    BMN --> BAT_PCT[/battery/percentage<br/>std_msgs/Float32]
    BMN --> BAT_STAT[/battery/status<br/>std_msgs/String]
    BMN --> LOW_BAT[/battery/low_battery_warning<br/>std_msgs/Bool]
    BMN --> CRIT_BAT[/battery/critical_battery_warning<br/>std_msgs/Bool]
    BMN <--> INA219

    %% TF Transforms
    MCN --> TF[TF: odom → base_link]

    %% Future Expansion (Planned)
    subgraph "Future Nodes (Planned)"
        LIDAR_N[lidar_node]
        CAM_N[camera_node]
        IMU_N[imu_node]
        SLAM_N[slam_node]
        NAV_N[navigation_node]
        WEB_N[web_interface_node]
        ARUCO_N[aruco_docking_node]
    end

    %% Future Topics (Planned)
    LIDAR_N -.-> SCAN[/scan<br/>sensor_msgs/LaserScan]
    CAM_N -.-> IMG[/camera/image_raw<br/>sensor_msgs/Image]
    IMU_N -.-> IMU_DATA[/imu/data<br/>sensor_msgs/Imu]
    SLAM_N -.-> MAP[/map<br/>nav_msgs/OccupancyGrid]
    WEB_N -.-> GOAL[/goal_pose<br/>geometry_msgs/PoseStamped]

    %% Styling
    classDef nodeStyle fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef topicStyle fill:#f3e5f5,stroke:#4a148c,stroke-width:1px
    classDef hardwareStyle fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef futureStyle fill:#fff3e0,stroke:#e65100,stroke-width:1px,stroke-dasharray: 5 5

    class MCN,ODN,BMN nodeStyle
    class CMD,ODOM,JOINTS,BAT_VOLT,STATUS,IP_ADDR,BAT_STATE,JET_BAT,BAT_V,BAT_C,BAT_P,BAT_PCT,BAT_STAT,LOW_BAT,CRIT_BAT,DISP_MSG topicStyle
    class RP2040,OLED,INA219,MOTORS,SERVO hardwareStyle
    class LIDAR_N,CAM_N,IMU_N,SLAM_N,NAV_N,WEB_N,ARUCO_N,SCAN,IMG,IMU_DATA,MAP,GOAL futureStyle
```

---

## 🚀 Launch Commands for Testing

### 1. Motor Control System
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Launch motor control
ros2 launch motor_control_pkg motor_control.launch.py

# Test robot movement
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'
```

### 2. OLED Display System
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Launch OLED display
ros2 launch oled_display_pkg oled_display.launch.py

# Send custom message
ros2 topic pub /oled_display_message std_msgs/String '{data: "Hello Robot!"}'
```

### 3. Battery Monitoring System
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Launch battery monitor
ros2 launch battery_monitor_pkg battery_monitor.launch.py

# Monitor battery data
ros2 topic echo /battery/percentage
ros2 topic echo /battery/voltage
```

### 4. RPLidar A1 System
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Option A: Basic RPLidar launch
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyACM1

# Option B: RPLidar with RViz (Recommended)
ros2 launch rplidar_ros view_rplidar_a1_launch.py serial_port:=/dev/ttyACM1

# Option C: Waveshare JetRacer specific
ros2 launch rplidar_ros waveshare_jetracer_rplidar_launch.py

# Test scan data
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### 5. Web Interface System
```bash
cd /home/orin/Documents/jetros/Autonomous_robot
source install/setup.bash

# Launch complete web interface (HTTP + WebSocket servers)
ros2 launch web_interface_pkg web_interface.launch.py

# Custom ports (optional)
ros2 launch web_interface_pkg web_interface.launch.py web_port:=8080 websocket_port:=8765

# Access web interface
# Local: http://localhost:8080
# Network: http://<robot-ip>:8080
```

### 6. Manual RViz Setup (if needed)
```bash
# Terminal 1: Launch RPLidar
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyACM1

# Terminal 2: Launch RViz with correct config
ros2 run rviz2 rviz2 -d src/rplidar_ros/rviz/rplidar_ros.rviz

# OR: Launch RViz and manually set Fixed Frame to 'laser'
ros2 run rviz2 rviz2
# In RViz: Global Options → Fixed Frame → type 'laser'
```

## 🔄 Quick Reference Commands

### Topic Monitoring
```bash
# List all active topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /battery/percentage
ros2 topic echo /robot_ip_address
ros2 topic echo /scan --once

# Check topic information
ros2 topic info /cmd_vel
ros2 topic hz /odom
ros2 topic hz /scan
```

### Node Management
```bash
# List all active nodes
ros2 node list

# Get node information
ros2 node info /motor_control_node
ros2 node info /battery_monitor_node
ros2 node info /oled_display_node
ros2 node info /rplidar_node
```

### System Testing
```bash
# Test robot movement
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'

# Send custom display message
ros2 topic pub /oled_display_message std_msgs/String '{data: "Hello Robot!"}'

# Check TF tree
ros2 run tf2_tools view_frames.py

# Check available launch files
ros2 launch rplidar_ros --show-args view_rplidar_a1_launch.py
```

## ⚠️ Important RViz Configuration Notes

### RPLidar RViz Setup
- **Issue**: Manual RViz defaults to 'map' frame, lidar data not visible
- **Solution**: Change Fixed Frame from 'map' to 'laser' (manual typing required)
- **Why**: RPLidar publishes in 'laser' frame, RViz needs matching reference frame
- **Best Practice**: Use `view_rplidar_a1_launch.py` for pre-configured setup

---

## 📝 Recent Updates

- **2025-06-29**: ✅ **CRITICAL FIX**: Resolved JSON parse errors in web interface caused by NaN values from battery monitor
- **2025-06-29**: ✅ Added data cleaning function in ROS bridge to handle NaN/Inf values before JSON serialization
- **2025-06-29**: ✅ Lidar data now flows correctly to web interface with real-time visualization
- **2025-06-29**: ✅ Enhanced debugging with Lidar Data Log Window for troubleshooting
- **2025-06-29**: ✅ Created comprehensive lidar troubleshooting guide (`LIDAR_TROUBLESHOOTING_GUIDE.md`)
- **2025-06-28**: Added comprehensive launch file for complete system integration
- **2025-06-28**: Integrated RPLidar A1 with web interface for real-time visualization
- **2025-06-28**: Enhanced web interface with lidar scan display and controls

---

**Note**: This documentation should be updated whenever new nodes, topics, or services are added to the system. All future agents should reference and maintain this file.
