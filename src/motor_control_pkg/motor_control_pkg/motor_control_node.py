#!/usr/bin/env python3
"""
Motor Control Node for Waveshare JetRacer
Interfaces between ROS2 and RP2040 secondary controller for motor and servo control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

from .waveshare_interface import WaveshareRP2040Interface


class MotorControlNode(Node):
    """ROS2 node for motor and servo control using Waveshare binary protocol"""
    
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.16)  # Distance between front and rear axles (m)
        self.declare_parameter('wheel_radius', 0.033)  # Wheel radius (m)
        self.declare_parameter('encoder_resolution', 1440)  # Encoder counts per revolution
        self.declare_parameter('max_speed', 1.0)  # Maximum speed (m/s)
        self.declare_parameter('max_steering_angle', 30.0)  # Maximum steering angle (degrees)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        
        # Initialize Waveshare interface
        self.rp2040 = WaveshareRP2040Interface(
            port=self.serial_port,
            baudrate=self.baudrate
        )
        
        # Set up callbacks
        self.rp2040.set_encoder_callback(self.encoder_callback)
        self.rp2040.set_status_callback(self.status_callback)
        
        # Connect to RP2040
        if not self.rp2040.connect():
            self.get_logger().error("Failed to connect to RP2040")
            raise RuntimeError("RP2040 connection failed")
        
        self.get_logger().info("Connected to RP2040 successfully")
        
        # ROS2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ROS2 publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.last_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_encoder_last = 0
        self.right_encoder_last = 0
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        self.get_logger().info("Motor control node initialized successfully")
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages using official Waveshare protocol"""
        try:
            # Extract velocity components according to Waveshare documentation
            linear_x = msg.linear.x      # Forward/backward speed (m/s)
            linear_y = msg.linear.x      # Same as linear_x (as per official code)
            angular_z = msg.angular.z    # Steering angle (radians), NOT angular velocity!

            # Clamp to official Waveshare ranges
            linear_x = max(-1.2, min(1.2, linear_x))
            angular_z = max(-0.6, min(0.6, angular_z))

            # Send velocity command to RP2040 using official protocol
            success = self.rp2040.send_velocity_command(linear_x, linear_y, angular_z)

            if success:
                self.get_logger().info(
                    f"✅ Sent cmd_vel: linear={linear_x:.2f} m/s, steering={angular_z:.2f} rad"
                )
            else:
                self.get_logger().warn("❌ Failed to send velocity command to RP2040")

        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {e}")
    
    def encoder_callback(self, encoder_data):
        """Handle encoder data from RP2040"""
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            
            if dt > 0:
                # Calculate wheel distances
                left_encoder = encoder_data['left_encoder']
                right_encoder = encoder_data['right_encoder']
                
                left_delta = left_encoder - self.left_encoder_last
                right_delta = right_encoder - self.right_encoder_last
                
                # Convert encoder counts to distances
                left_distance = (left_delta / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
                right_distance = (right_delta / self.encoder_resolution) * 2 * math.pi * self.wheel_radius
                
                # Calculate robot motion
                distance = (left_distance + right_distance) / 2.0
                delta_theta = (right_distance - left_distance) / 0.165  # wheel separation
                
                # Update pose
                self.x += distance * math.cos(self.theta + delta_theta / 2.0)
                self.y += distance * math.sin(self.theta + delta_theta / 2.0)
                self.theta += delta_theta
                
                # Update last values
                self.left_encoder_last = left_encoder
                self.right_encoder_last = right_encoder
                self.last_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing encoder data: {e}")
    
    def status_callback(self, status_data):
        """Handle status data from RP2040"""
        try:
            # Publish battery voltage
            battery_msg = Float32()
            battery_msg.data = status_data.get('battery_voltage', 0.0)
            self.battery_pub.publish(battery_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = f"Motors: {status_data.get('motor_status', 'unknown')}, " \
                            f"Servo: {status_data.get('servo_status', 'unknown')}"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing status data: {e}")
    
    def timer_callback(self):
        """Timer callback to publish odometry and joint states"""
        try:
            current_time = self.get_clock().now()
            
            # Publish odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Orientation (quaternion from yaw)
            q = self._yaw_to_quaternion(self.theta)
            odom_msg.pose.pose.orientation = q
            
            # Velocity (simplified)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            
            self.odom_pub.publish(odom_msg)
            
            # Publish transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
    
    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def shutdown(self):
        """Shutdown the node"""
        self.get_logger().info("Shutting down motor control node...")
        self.rp2040.stop_motors()
        self.rp2040.disconnect()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = MotorControlNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.shutdown()
            
    except Exception as e:
        print(f"Failed to start motor control node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
