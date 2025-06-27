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

### oled_display_pkg
OLED display interface for showing IP address and system information.
- **I2C Bus**: 7 (GPIO pins 3,5)
- **Features**: IP display, custom messages via ROS2 topics
- **Status**: ✅ Complete and tested

### motor_control_pkg
Motor and servo control interface for Waveshare JetRacer with RP2040 secondary controller.
- **Communication**: Serial JSON protocol over USB
- **Features**: Differential drive, Ackerman steering, odometry, cmd_vel interface
- **Status**: ✅ Complete and tested (simulation mode)

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
source install/setup.bash```

2. **Launch OLED display**:
```bash
ros2 launch oled_display_pkg oled_with_params.launch.py
```

3. **Launch motor control (simulation)**:
```bash
ros2 run motor_control_pkg motor_control_node --ros-args -p simulation_mode:=true
```

4. **Test motor commands**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.5}}'
```

## Development Guidelines

- All packages go in `src/` folder

- Test thoroughly before committing

## Maintainer

Teja

## License

MIT License
