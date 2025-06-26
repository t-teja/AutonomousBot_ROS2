# OLED Display Package for Jetson Orin Nano

This ROS2 package provides OLED display functionality for the Waveshare JetRacer with a 128x32 SSD1306 OLED display connected via I2C.

## Features

- Display IP address on OLED
- Automatic IP address detection and refresh
- ROS2 topic interface for custom messages
- Configurable update rates and parameters
- Robust error handling and recovery

## Hardware Requirements

- Jetson Orin Nano with JetPack 6.2
- 128x32 SSD1306 OLED display
- I2C connection to Bus 7 (GPIO header pins 3,5) at address 0x3C

## I2C Bus Configuration

**IMPORTANT**: This package uses I2C Bus 7 on Jetson Orin Nano
- **Bus 7**: GPIO header pins 3 (SDA) and 5 (SCL) - Safe for external devices
- **Address**: 0x3C (OLED display)
- **Other devices on Bus 7**: 0x40, 0x41 (existing hardware)
- **AVOID Bus 5**: Used internally for CSI camera interface - scanning can cause system crashes

## Dependencies

- ROS2 Humble
- Python packages: smbus2, pillow, netifaces
- I2C tools (i2c-tools package)

## Installation

1. Build the package:
```bash
cd ~/Documents/jetros/Autonomous_robot
colcon build --packages-select oled_display_pkg
source install/setup.bash
```

2. Install Python dependencies:
```bash
pip3 install smbus2 pillow netifaces
```

3. Verify I2C connection:
```bash
sudo i2cdetect -y -r 7
# Should show device at 0x3C
```

## Hardware Connections

Connect OLED display to Jetson Orin Nano GPIO header:
- **VCC** → Pin 1 (3.3V)
- **GND** → Pin 6 (Ground)
- **SDA** → Pin 3 (I2C1_SDA)
- **SCL** → Pin 5 (I2C1_SCL)

## Usage

### Launch with Default Parameters
```bash
ros2 launch oled_display_pkg oled_with_params.launch.py
```

### Launch with Custom Parameters
```bash
ros2 launch oled_display_pkg oled_display.launch.py i2c_bus:=7 update_rate:=5.0
```

### Run Node Directly
```bash
ros2 run oled_display_pkg oled_display_node
```

## ROS2 Topics

### Published Topics
- `/robot_ip_address` (std_msgs/String): Current IP address

### Subscribed Topics
- `/oled_display_message` (std_msgs/String): Custom messages to display

## Parameters

- `i2c_bus`: I2C bus number (default: 7)
- `i2c_address`: I2C address (default: 0x3C)
- `update_rate`: Display refresh rate in Hz (default: 5.0)
- `ip_refresh_interval`: IP check interval in seconds (default: 30.0)

## Troubleshooting

### I2C Issues
```bash
# Check I2C devices on Bus 7
sudo i2cdetect -y -r 7

# Check I2C permissions
sudo usermod -a -G i2c $USER
# Logout and login again
```

### Display Not Working
- Verify I2C connections to GPIO header pins 3,5
- Check I2C address (should be 0x3C)
- Ensure proper power supply to OLED (3.3V)
- Check ROS2 logs for error messages

## Example Usage
```bash
# Send custom message to display
ros2 topic pub /oled_display_message std_msgs/String 'data: "Hello JetRacer!"' --once

# Check current IP being published
ros2 topic echo /robot_ip_address --once
```

## Maintainer

Teja

## License

MIT License
