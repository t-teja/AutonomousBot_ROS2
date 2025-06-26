#!/usr/bin/env python3
"""
Launch file for OLED Display Node with parameter file
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with parameter file"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('oled_display_pkg')
    
    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'oled_params.yaml')
    
    # OLED Display Node with parameters
    oled_display_node = Node(
        package='oled_display_pkg',
        executable='oled_display_node',
        name='oled_display_node',
        output='screen',
        parameters=[params_file],
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        oled_display_node,
    ])
