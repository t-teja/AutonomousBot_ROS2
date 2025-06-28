from setuptools import find_packages, setup

package_name = 'battery_monitor_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/battery_monitor.launch.py']),
    ],
    install_requires=['setuptools', 'pi-ina219'],
    zip_safe=True,
    maintainer='Teja',
    maintainer_email='tejalabs@outlook.com',
    description='Battery monitoring package for INA219 sensor on Waveshare JetRacer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor_node = battery_monitor_pkg.battery_monitor_node:main',
        ],
    },
)
