#!/usr/bin/env python3
"""
Complete JetRacer System Launch File
Launches all robot nodes + web interface for full system testing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for complete JetRacer system"""
    
    # Declare launch arguments
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Launch RPLidar A1 (default: true)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for lidar visualization (default: false)'
    )
    
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
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes (debug, info, warn, error)'
    )
    
    # System startup message
    startup_log = LogInfo(
        msg=[
            '\n',
            '🤖 Starting Complete JetRacer Autonomous Robot System\n',
            '=' * 60, '\n',
            '📋 System Components:\n',
            '  ✅ Motor Control (RP2040 + Encoders + Servo)\n',
            '  ✅ OLED Display (IP Address + Messages)\n',
            '  ✅ Battery Monitor (INA219 Voltage/Current/Power)\n',
            '  ✅ RPLidar A1 (Laser Scanner)\n',
            '  ✅ Web Interface (Mobile/Tablet Control)\n',
            '\n',
            '🌐 Web Interface Access:\n',
            '  Local:   http://localhost:', LaunchConfiguration('web_port'), '\n',
            '  Network: http://<robot-ip>:', LaunchConfiguration('web_port'), '\n',
            '\n',
            '📱 Features Available:\n',
            '  • Real-time robot control via web interface\n',
            '  • Battery monitoring dashboard\n',
            '  • Lidar scan visualization\n',
            '  • System status monitoring\n',
            '  • OLED display control\n',
            '  • Mobile/tablet optimized interface\n',
            '\n',
            '⚠️  Hardware Requirements:\n',
            '  • RP2040 controller on /dev/ttyACM0\n',
            '  • RPLidar A1 on /dev/ttyACM1\n',
            '  • OLED display on I2C bus 7\n',
            '  • INA219 battery monitor on I2C bus 7\n',
            '\n',
            '🚀 Starting all nodes...\n',
            '=' * 60
        ]
    )
    
    # 1. Motor Control Node (Working Backup Implementation)
    motor_control_node = ExecuteProcess(
        cmd=[
            'python3',
            '/home/orin/Documents/jetros/jetracer_project_backup-27-06-25/Jetracer/scripts/jetracer_servo_node_with_encoders.py',
            '--ros-args',
            '-p', 'serial_port:=/dev/ttyACM0',
            '-p', 'baud_rate:=115200',
            '-p', 'max_steering_angle:=0.6',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # 2. OLED Display Node
    oled_display_node = Node(
        package='oled_display_pkg',
        executable='oled_display_node',
        name='oled_display_node',
        output='screen',
        parameters=[{
            'i2c_bus': 7,
            'i2c_address': 0x3C,
            'update_rate': 2.0,
            'ip_refresh_interval': 30.0,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )

    # 3. Battery Monitor Node
    battery_monitor_node = Node(
        package='battery_monitor_pkg',
        executable='battery_monitor_node',
        name='battery_monitor_node',
        output='screen',
        parameters=[{
            'i2c_bus': 7,
            'i2c_address': 0x41,
            'publish_rate': 1.0,
            'nominal_voltage': 11.1,
            'max_voltage': 12.6,
            'min_voltage': 9.0,
            'capacity': 7.8,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        respawn=True,
        respawn_delay=2.0
    )
    
    # 4. RPLidar Node (conditional)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyACM1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('use_lidar')),
        respawn=True,
        respawn_delay=3.0
    )
    
    # 5. Static Transform (base_link to laser)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # 6. RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/rplidar_ros/rviz/rplidar_ros.rviz'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # 7. Web Server Node (delayed start)
    web_server_node = TimerAction(
        period=3.0,  # Wait 3 seconds for other nodes to start
        actions=[
            Node(
                package='web_interface_pkg',
                executable='web_server_node.py',
                name='web_server',
                output='screen',
                parameters=[{
                    'port': LaunchConfiguration('web_port'),
                    'host': '0.0.0.0',
                }],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
            )
        ]
    )
    
    # 8. ROS Bridge Node (delayed start)
    ros_bridge_node = TimerAction(
        period=3.0,  # Wait 3 seconds for other nodes to start
        actions=[
            Node(
                package='web_interface_pkg',
                executable='ros_bridge_node.py',
                name='ros_bridge',
                output='screen',
                parameters=[{
                    'websocket_port': LaunchConfiguration('websocket_port'),
                    'host': '0.0.0.0',
                    'publish_rate': 10.0,
                    'max_linear_velocity': 1.0,
                    'max_angular_velocity': 2.0,
                    'command_timeout': 1.0,
                }],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
            )
        ]
    )
    
    # Final startup message
    ready_log = TimerAction(
        period=5.0,
        actions=[
            LogInfo(
                msg=[
                    '\n',
                    '🎉 JetRacer System Startup Complete!\n',
                    '=' * 50, '\n',
                    '🌐 Web Interface Ready:\n',
                    '  URL: http://localhost:', LaunchConfiguration('web_port'), '\n',
                    '  Mobile: http://<robot-ip>:', LaunchConfiguration('web_port'), '\n',
                    '\n',
                    '📊 Expected Data in Web Interface:\n',
                    '  • Battery: Real voltage, current, percentage\n',
                    '  • Robot Status: Motor control status\n',
                    '  • Lidar: Real-time scan visualization\n',
                    '  • System: CPU, memory, temperature\n',
                    '\n',
                    '🎮 Controls Available:\n',
                    '  • Virtual joystick for robot movement\n',
                    '  • Emergency stop button\n',
                    '  • OLED display messaging\n',
                    '  • Lidar start/stop controls\n',
                    '\n',
                    '✅ All systems operational!\n',
                    '=' * 50
                ]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_lidar_arg,
        use_rviz_arg,
        web_port_arg,
        websocket_port_arg,
        log_level_arg,
        
        # Startup message
        startup_log,
        
        # Core robot nodes (start immediately)
        motor_control_node,
        oled_display_node,
        battery_monitor_node,
        
        # Lidar nodes (conditional)
        rplidar_node,
        base_to_laser_tf,
        rviz_node,
        
        # Web interface nodes (delayed start)
        web_server_node,
        ros_bridge_node,
        
        # Ready message
        ready_log,
    ])
