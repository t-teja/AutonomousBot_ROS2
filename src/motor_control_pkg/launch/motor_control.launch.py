#!/usr/bin/env python3
"""
Launch file for Motor Control System
Starts motor control node with configurable parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for motor control system"""
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for RP2040 communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial communication baud rate'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.16',
        description='Distance between front and rear axles (m)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Maximum linear speed (m/s)'
    )
    
    max_steering_angle_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='30.0',
        description='Maximum steering angle (degrees)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Motor Control Node
    motor_control_node = Node(
        package='motor_control_pkg',
        executable='motor_control_node',
        name='motor_control_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Log startup information
    startup_log = LogInfo(
        msg=[
            'Starting Motor Control System with parameters:\n',
            '  Serial Port: ', LaunchConfiguration('serial_port'), '\n',
            '  Baud Rate: ', LaunchConfiguration('baudrate'), '\n',
            '  Wheel Base: ', LaunchConfiguration('wheel_base'), ' m\n',
            '  Max Speed: ', LaunchConfiguration('max_speed'), ' m/s\n',
            '  Max Steering Angle: ', LaunchConfiguration('max_steering_angle'), ' degrees\n',
            '  Log Level: ', LaunchConfiguration('log_level')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baudrate_arg,
        wheel_base_arg,
        max_speed_arg,
        max_steering_angle_arg,
        log_level_arg,
        
        # Log startup info
        startup_log,
        
        # Nodes
        motor_control_node,
    ])
