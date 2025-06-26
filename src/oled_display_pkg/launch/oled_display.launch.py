#!/usr/bin/env python3
"""
Launch file for OLED Display Node
Configures and starts the OLED display system for Jetson Orin Nano
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate launch description for OLED display system"""
    
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for OLED display (GPIO header pins 3,5)'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x3C',
        description='I2C address of OLED display (hex format)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='5.0',
        description='Display update rate in Hz'
    )
    
    ip_refresh_interval_arg = DeclareLaunchArgument(
        'ip_refresh_interval',
        default_value='30.0',
        description='IP address refresh interval in seconds'
    )
    

    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Convert hex string to integer for i2c_address
    i2c_address_int = LaunchConfiguration('i2c_address')
    
    # OLED Display Node
    oled_display_node = Node(
        package='oled_display_pkg',
        executable='oled_display_node',
        name='oled_display_node',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': 0x3C,
            'update_rate': LaunchConfiguration('update_rate'),
            'ip_refresh_interval': LaunchConfiguration('ip_refresh_interval'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Log startup information
    startup_log = LogInfo(
        msg=[
            'Starting OLED Display System with parameters:\n',
            '  I2C Bus: ', LaunchConfiguration('i2c_bus'), '\n',
            '  I2C Address: ', LaunchConfiguration('i2c_address'), '\n',
            '  Update Rate: ', LaunchConfiguration('update_rate'), ' Hz\n',
            '  IP Refresh Interval: ', LaunchConfiguration('ip_refresh_interval'), ' seconds\n',
            '  Display Mode: ', LaunchConfiguration('display_mode'), '\n',
            '  Log Level: ', LaunchConfiguration('log_level')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        i2c_bus_arg,
        i2c_address_arg,
        update_rate_arg,
        ip_refresh_interval_arg,
        log_level_arg,
        
        # Log startup info
        startup_log,
        
        # Nodes
        oled_display_node,
    ])
