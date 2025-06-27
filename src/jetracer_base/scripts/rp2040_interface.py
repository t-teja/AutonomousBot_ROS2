#!/usr/bin/env python3
"""
RP2040 Interface Node for Jetracer
Handles communication with the RP2040 microcontroller via USB and I2C
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import json
import time
from threading import Lock

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from jetracer_msgs.msg import MotorState, MotorCommand, SystemStatus, WheelSpeeds


class RP2040Interface(Node):
    def __init__(self):
        super().__init__('rp2040_interface')
        
        # Parameters
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        self.usb_port = self.get_parameter('usb_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        
        # Publishers
        self.motor_state_pub = self.create_publisher(MotorState, '/motor_states', 10)
        self.wheel_speeds_pub = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.system_status_pub = self.create_publisher(SystemStatus, '/system_status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_cmd_sub = self.create_subscription(
            MotorCommand, '/motor_command', self.motor_command_callback, 10)
        
        # Timer for publishing sensor data
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Initialize connection
        self.connect_to_rp2040()
        
        self.get_logger().info(f'RP2040 Interface initialized on {self.usb_port}')
    
    def connect_to_rp2040(self):
        """Establish connection to RP2040"""
        try:
            self.serial_conn = serial.Serial(
                port=self.usb_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.get_logger().info('Connected to RP2040')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to RP2040: {e}')
            return False
    
    def send_command(self, command_dict):
        """Send command to RP2040"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not available')
            return False
        
        try:
            with self.serial_lock:
                command_json = json.dumps(command_dict) + '\n'
                self.serial_conn.write(command_json.encode())
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False
    
    def read_response(self):
        """Read response from RP2040"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            with self.serial_lock:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        return json.loads(line)
        except Exception as e:
            self.get_logger().error(f'Failed to read response: {e}')
        
        return None
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        # Convert twist to differential drive commands
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Differential drive kinematics
        # Assuming wheelbase of 0.165m (from config)
        wheelbase = 0.165
        wheel_radius = 0.0325  # 65mm diameter / 2
        
        # Calculate wheel speeds
        left_wheel_speed = (linear_vel - angular_vel * wheelbase / 2) / wheel_radius
        right_wheel_speed = (linear_vel + angular_vel * wheelbase / 2) / wheel_radius
        
        # Send motor commands
        command = {
            'type': 'motor_control',
            'left_motor': {
                'speed': left_wheel_speed,
                'enable': True
            },
            'right_motor': {
                'speed': right_wheel_speed,
                'enable': True
            }
        }
        
        self.send_command(command)
    
    def motor_command_callback(self, msg):
        """Handle direct motor commands"""
        command = {
            'type': 'motor_command',
            'motor_name': msg.motor_name,
            'control_mode': msg.control_mode,
            'velocity': msg.velocity,
            'position': msg.position,
            'effort': msg.effort,
            'pwm_duty_cycle': msg.pwm_duty_cycle,
            'enable': msg.enable,
            'emergency_stop': msg.emergency_stop
        }
        
        self.send_command(command)
    
    def timer_callback(self):
        """Timer callback to read sensor data"""
        # Request sensor data
        request = {'type': 'get_sensor_data'}
        if self.send_command(request):
            # Read response
            response = self.read_response()
            if response:
                self.process_sensor_data(response)
    
    def process_sensor_data(self, data):
        """Process sensor data from RP2040"""
        try:
            if 'motor_states' in data:
                self.publish_motor_states(data['motor_states'])
            
            if 'wheel_speeds' in data:
                self.publish_wheel_speeds(data['wheel_speeds'])
            
            if 'system_status' in data:
                self.publish_system_status(data['system_status'])
                
        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {e}')
    
    def publish_motor_states(self, motor_data):
        """Publish motor state messages"""
        for motor_name, state in motor_data.items():
            msg = MotorState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.motor_name = motor_name
            msg.velocity = state.get('velocity', 0.0)
            msg.position = state.get('position', 0.0)
            msg.effort = state.get('effort', 0.0)
            msg.current = state.get('current', 0.0)
            msg.temperature = state.get('temperature', 0.0)
            msg.encoder_ticks = state.get('encoder_ticks', 0)
            msg.encoder_velocity = state.get('encoder_velocity', 0.0)
            msg.command_velocity = state.get('command_velocity', 0.0)
            msg.pid_output = state.get('pid_output', 0.0)
            msg.pwm_duty_cycle = state.get('pwm_duty_cycle', 0.0)
            msg.is_enabled = state.get('is_enabled', False)
            msg.has_error = state.get('has_error', False)
            msg.error_message = state.get('error_message', '')
            
            self.motor_state_pub.publish(msg)
    
    def publish_wheel_speeds(self, wheel_data):
        """Publish wheel speeds message"""
        msg = WheelSpeeds()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left_wheel_speed = wheel_data.get('left_speed', 0.0)
        msg.right_wheel_speed = wheel_data.get('right_speed', 0.0)
        msg.left_wheel_command = wheel_data.get('left_command', 0.0)
        msg.right_wheel_command = wheel_data.get('right_command', 0.0)
        msg.left_wheel_error = wheel_data.get('left_error', 0.0)
        msg.right_wheel_error = wheel_data.get('right_error', 0.0)
        msg.left_wheel_position = wheel_data.get('left_position', 0.0)
        msg.right_wheel_position = wheel_data.get('right_position', 0.0)
        msg.left_encoder_ticks = wheel_data.get('left_ticks', 0)
        msg.right_encoder_ticks = wheel_data.get('right_ticks', 0)
        
        self.wheel_speeds_pub.publish(msg)
    
    def publish_system_status(self, status_data):
        """Publish system status message"""
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_name = "jetracer"
        msg.hardware_version = "1.0"
        msg.software_version = "1.0"
        msg.system_state = status_data.get('state', 0)
        msg.status_message = status_data.get('message', '')
        msg.rp2040_connected = True
        
        self.system_status_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RP2040Interface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
