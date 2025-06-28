#!/usr/bin/env python3
"""
Launch file for Battery Monitor Node
Starts INA219 battery monitoring with proper configuration for Waveshare JetRacer
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for battery monitor"""
    
    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number (7 for external devices on Jetson Orin)'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='65',  # 0x41 in decimal
        description='INA219 I2C address (65 = 0x41)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Battery data publish rate in Hz'
    )
    
    # Battery configuration arguments
    nominal_voltage_arg = DeclareLaunchArgument(
        'nominal_voltage',
        default_value='11.1',
        description='Battery nominal voltage (3x 18650 Li-ion = 11.1V)'
    )
    
    max_voltage_arg = DeclareLaunchArgument(
        'max_voltage',
        default_value='12.6',
        description='Battery maximum voltage (3x 4.2V = 12.6V)'
    )
    
    min_voltage_arg = DeclareLaunchArgument(
        'min_voltage',
        default_value='9.0',
        description='Battery minimum voltage (3x 3.0V = 9.0V)'
    )
    
    capacity_arg = DeclareLaunchArgument(
        'capacity',
        default_value='7.8',
        description='Battery capacity in Ah (3x 2600mAh = 7.8Ah)'
    )
    
    # Warning threshold arguments
    low_battery_threshold_arg = DeclareLaunchArgument(
        'low_battery_threshold',
        default_value='20.0',
        description='Low battery warning threshold in percentage'
    )
    
    critical_battery_threshold_arg = DeclareLaunchArgument(
        'critical_battery_threshold',
        default_value='10.0',
        description='Critical battery warning threshold in percentage'
    )
    
    # Battery monitor node
    battery_monitor_node = Node(
        package='battery_monitor_pkg',
        executable='battery_monitor_node',
        name='battery_monitor',
        output='screen',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'shunt_ohms': 0.05,
            'max_expected_amps': 3.2,
            'nominal_voltage': LaunchConfiguration('nominal_voltage'),
            'max_voltage': LaunchConfiguration('max_voltage'),
            'min_voltage': LaunchConfiguration('min_voltage'),
            'capacity': LaunchConfiguration('capacity'),
            'low_battery_threshold': LaunchConfiguration('low_battery_threshold'),
            'critical_battery_threshold': LaunchConfiguration('critical_battery_threshold'),
            'voltage_filter_alpha': 0.9,
            'current_filter_alpha': 0.8,
        }],
        remappings=[
            # Standard battery topics
            ('battery_state', '/battery_state'),
            ('battery/voltage', '/battery/voltage'),
            ('battery/current', '/battery/current'),
            ('battery/power', '/battery/power'),
            ('battery/percentage', '/battery/percentage'),
            ('battery/status', '/battery/status'),
            ('battery/low_battery_warning', '/battery/low_battery_warning'),
            ('battery/critical_battery_warning', '/battery/critical_battery_warning'),
            # JetRacer specific topics
            ('jetracer/battery_state', '/jetracer/battery_state'),
        ]
    )
    
    return LaunchDescription([
        i2c_bus_arg,
        i2c_address_arg,
        publish_rate_arg,
        nominal_voltage_arg,
        max_voltage_arg,
        min_voltage_arg,
        capacity_arg,
        low_battery_threshold_arg,
        critical_battery_threshold_arg,
        battery_monitor_node,
    ])
