#!/usr/bin/env python3
"""
Servo Controller Node for Jetracer
Handles servo steering control via RP2040 controller
"""

import rclpy
from rclpy.node import Node
import serial
import json
import time
from threading import Lock

from geometry_msgs.msg import Twist
from jetracer_msgs.msg import ServoCommand, ServoState


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parameters
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        
        # Servo configuration parameters
        self.declare_parameter('center_position', 90.0)
        self.declare_parameter('max_left_angle', 45.0)
        self.declare_parameter('max_right_angle', 135.0)
        self.declare_parameter('max_steering_angle', 30.0)  # Max steering from center
        self.declare_parameter('servo_speed', 100.0)
        
        # Get parameters
        self.usb_port = self.get_parameter('usb_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.center_position = self.get_parameter('center_position').value
        self.max_left_angle = self.get_parameter('max_left_angle').value
        self.max_right_angle = self.get_parameter('max_right_angle').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.servo_speed = self.get_parameter('servo_speed').value
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        
        # Current servo state
        self.current_angle = self.center_position
        self.target_angle = self.center_position
        self.last_command_time = time.time()
        
        # Publishers
        self.servo_state_pub = self.create_publisher(ServoState, '/servo_state', 10)
        
        # Subscribers
        self.servo_cmd_sub = self.create_subscription(
            ServoCommand, '/servo_command', self.servo_command_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer for publishing servo state
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # Initialize connection
        self.connect_to_rp2040()
        
        # Set servo to center position on startup
        self.set_servo_angle(self.center_position)
        
        self.get_logger().info(f'Servo Controller initialized on {self.usb_port}')
        self.get_logger().info(f'Servo range: {self.max_left_angle}° to {self.max_right_angle}°')
    
    def connect_to_rp2040(self):
        """Establish connection to RP2040"""
        try:
            self.serial_conn = serial.Serial(
                port=self.usb_port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.get_logger().info('Connected to RP2040 for servo control')
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
            self.get_logger().error(f'Failed to send servo command: {e}')
            return False
    
    def set_servo_angle(self, angle_degrees):
        """Set servo to specific angle"""
        # Clamp angle to valid range
        angle_degrees = max(self.max_left_angle, 
                           min(self.max_right_angle, angle_degrees))
        
        # Create servo command
        command = {
            'type': 'servo_control',
            'servo_name': 'steering',
            'angle': angle_degrees,
            'speed': self.servo_speed
        }
        
        if self.send_command(command):
            self.target_angle = angle_degrees
            self.last_command_time = time.time()
            self.get_logger().info(f'Servo commanded to {angle_degrees:.1f}°')
            return True
        
        return False
    
    def servo_command_callback(self, msg):
        """Handle direct servo commands"""
        if msg.servo_name == 'steering' or msg.servo_name == '':
            if msg.enable and not msg.emergency_stop:
                # Validate angle range
                if msg.min_angle != 0.0 and msg.max_angle != 0.0:
                    angle = max(msg.min_angle, min(msg.max_angle, msg.angle))
                else:
                    angle = max(self.max_left_angle, 
                               min(self.max_right_angle, msg.angle))
                
                self.set_servo_angle(angle)
            elif msg.emergency_stop:
                # Emergency stop - return to center
                self.set_servo_angle(self.center_position)
                self.get_logger().warn('Servo emergency stop - returning to center')
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages for steering"""
        # Convert angular velocity to steering angle
        # Positive angular.z = turn left (decrease angle)
        # Negative angular.z = turn right (increase angle)

        angular_vel = msg.angular.z

        # Add debug logging
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={angular_vel:.2f}')

        # Scale angular velocity to steering angle
        # Assuming max angular velocity of 1.0 rad/s maps to max steering
        max_angular_vel = 1.0  # rad/s

        # Calculate steering angle from center
        steering_offset = -angular_vel * self.max_steering_angle / max_angular_vel
        target_angle = self.center_position + steering_offset

        # Clamp to servo limits
        target_angle = max(self.max_left_angle,
                          min(self.max_right_angle, target_angle))

        self.get_logger().info(f'Calculated target angle: {target_angle:.1f}° (offset: {steering_offset:.1f}°)')

        self.set_servo_angle(target_angle)
    
    def timer_callback(self):
        """Timer callback to publish servo state"""
        # Simulate servo movement (in real implementation, read from RP2040)
        time_since_command = time.time() - self.last_command_time
        
        # Simple servo movement simulation
        if abs(self.current_angle - self.target_angle) > 0.5:
            # Servo is moving towards target
            direction = 1 if self.target_angle > self.current_angle else -1
            servo_speed_deg_per_sec = 60.0  # degrees per second
            movement = direction * servo_speed_deg_per_sec / self.publish_rate
            self.current_angle += movement
            
            # Don't overshoot
            if direction > 0:
                self.current_angle = min(self.current_angle, self.target_angle)
            else:
                self.current_angle = max(self.current_angle, self.target_angle)
        
        # Publish servo state
        self.publish_servo_state()
    
    def publish_servo_state(self):
        """Publish current servo state"""
        msg = ServoState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_name = 'steering'
        msg.current_angle = self.current_angle
        msg.target_angle = self.target_angle
        msg.angular_velocity = 0.0  # TODO: Calculate from position change
        msg.is_moving = abs(self.current_angle - self.target_angle) > 0.5
        msg.is_enabled = True
        msg.has_error = False
        msg.error_message = ''
        msg.min_angle = self.max_left_angle
        msg.max_angle = self.max_right_angle
        msg.max_speed = 100.0
        msg.pwm_value = 0.0  # TODO: Calculate PWM value
        msg.command_timestamp = self.last_command_time
        
        self.servo_state_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        # Return servo to center position
        self.set_servo_angle(self.center_position)
        time.sleep(0.5)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
