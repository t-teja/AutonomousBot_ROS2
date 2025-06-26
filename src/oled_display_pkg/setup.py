import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'oled_display_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'smbus2', 'pillow', 'netifaces'],
    zip_safe=True,
    maintainer='Teja',
    maintainer_email='tejalabs@outlook.com',
    description='ROS2 package for controlling 128x32 OLED display via I2C to show IP address on Jetson Orin Nano',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oled_display_node = oled_display_pkg.oled_display_node:main',
        ],
    },
)
