from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teja',
    maintainer_email='tejalabs@outlook.com',
    description='Motor control package for Waveshare JetRacer with RP2040',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_pkg.motor_control_node:main',
        ],
    },
)
