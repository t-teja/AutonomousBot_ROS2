#!/usr/bin/env python3
"""
Launch file for JetRacer Web Interface
Starts both the web server and ROS bridge nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for web interface system"""
    
    # Declare launch arguments
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for web server (default: 8080)'
    )
    
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='8765',
        description='Port for WebSocket server (default: 8765)'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address to bind servers (default: 0.0.0.0 for all interfaces)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Web Server Node
    web_server_node = Node(
        package='web_interface_pkg',
        executable='web_server_node.py',
        name='web_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('web_port'),
            'host': LaunchConfiguration('host'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # ROS Bridge Node (WebSocket server)
    ros_bridge_node = Node(
        package='web_interface_pkg',
        executable='ros_bridge_node.py',
        name='ros_bridge',
        output='screen',
        parameters=[{
            'websocket_port': LaunchConfiguration('websocket_port'),
            'host': LaunchConfiguration('host'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Startup information
    startup_log = LogInfo(
        msg=[
            'Starting JetRacer Web Interface:\n',
            '  Web Server: http://', LaunchConfiguration('host'), ':', LaunchConfiguration('web_port'), '\n',
            '  WebSocket: ws://', LaunchConfiguration('host'), ':', LaunchConfiguration('websocket_port'), '\n',
            '  Log Level: ', LaunchConfiguration('log_level'), '\n',
            '\n',
            'Access the robot control interface at:\n',
            '  Local: http://localhost:', LaunchConfiguration('web_port'), '\n',
            '  Network: http://<robot-ip>:', LaunchConfiguration('web_port'), '\n',
            '\n',
            'Features:\n',
            '  - Real-time robot control with touch/mouse\n',
            '  - Battery and system monitoring\n',
            '  - Lidar visualization\n',
            '  - Mobile and tablet optimized\n',
            '  - Progressive Web App (PWA) support'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        web_port_arg,
        websocket_port_arg,
        host_arg,
        log_level_arg,
        
        # Startup log
        startup_log,
        
        # Nodes
        web_server_node,
        ros_bridge_node,
    ])
