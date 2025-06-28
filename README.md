# Autonomous Robot Project

This is the main ROS2 workspace for an autonomous robot project using Jetson Orin Nano with Waveshare JetRacer hardware.

## Hardware Platform

- **Main Controller**: Jetson Orin Nano with JetPack 6.2
- **Secondary Controller**: RP2040 (Waveshare JetRacer)
- **Motors**: 2x rear DC motors with encoders
- **Steering**: Front Ackerman steering servo
- **Sensors**:
  - RPLidar A1 (USB)
  - IMX219 camera (CSI interface)
  - 9520 IMU (onboard)
- **Display**: 128x32 OLED display (I2C)
- **Audio**: USB speaker and microphone

## Software Stack

- **OS**: Ubuntu with JetPack 6.2
- **ROS**: ROS2 Humble
- **Programming**: Python 3.10

## Packages

### motor_control_pkg
Motor and servo control interface for Waveshare JetRacer with RP2040 secondary controller.
- **Communication**: Binary protocol over USB (/dev/ttyACM0)
- **Features**: Differential drive, Ackerman steering, odometry, cmd_vel interface
- **Status**: ✅ Complete and tested

### oled_display_pkg
OLED display interface for showing IP address and system information.
- **I2C Bus**: 7 (GPIO pins 3,5)
- **Features**: IP display, custom messages via ROS2 topics
- **Status**: ✅ Complete and tested

### battery_monitor_pkg
INA219 battery monitoring system for power management.
- **I2C Bus**: 7, Address: 0x41
- **Features**: Voltage, current, power, percentage monitoring with warnings
- **Status**: ✅ Complete and tested

### rplidar_ros
RPLidar A1 laser scanner integration for SLAM and navigation.
- **Hardware**: RPLidar A1M8 via Waveshare JetRacer board (/dev/ttyACM1)
- **Features**: Real-time laser scan data, RViz visualization
- **Status**: ✅ Complete and tested

### jetracer_msgs
Custom ROS2 message definitions for JetRacer communication.
- **Messages**: BatteryState, MotorCommand, MotorState, ServoCommand, etc.
- **Services**: Calibration and PID tuning services
- **Status**: ✅ Complete and working

## I2C Bus Configuration Notes

**IMPORTANT**: Jetson Orin Nano I2C bus usage:
- **Bus 7**: GPIO header pins 3,5 - Safe for external devices (OLED display)
- **Bus 1**: Internal devices at 0x25, 0x40
- **Bus 5**: CSI camera interface - DO NOT SCAN (can cause system crashes)

## Quick Start

1. **Build workspace**:
```bash
cd Autonomous_robot
colcon build
source install/setup.bash
```

2. **Launch Motor Control**:
```bash
ros2 launch motor_control_pkg motor_control.launch.py
```

3. **Launch OLED Display**:
```bash
ros2 launch oled_display_pkg oled_display.launch.py
```

4. **Launch Battery Monitor**:
```bash
ros2 launch battery_monitor_pkg battery_monitor.launch.py
```

5. **Launch RPLidar with RViz**:
```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py serial_port:=/dev/ttyACM1
```

6. **Test Commands**:
```bash
# Test robot movement
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'

# Test OLED display
ros2 topic pub /oled_display_message std_msgs/String '{data: "Hello Robot!"}'

# Monitor battery
ros2 topic echo /battery/percentage

# Monitor lidar
ros2 topic hz /scan
```

## RViz Configuration Notes

**Important**: When launching RViz manually for lidar visualization:
- Change Fixed Frame from 'map' to 'laser' (manual typing required)
- Add LaserScan display with topic '/scan'
- Or use the pre-configured launch: `view_rplidar_a1_launch.py`

## Development Guidelines

- All packages go in `src/` folder

- Test thoroughly before committing

## Maintainer

Teja

## License

MIT License
