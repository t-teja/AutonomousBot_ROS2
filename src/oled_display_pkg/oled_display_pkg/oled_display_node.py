#!/usr/bin/env python3
"""
ROS2 OLED Display Node for Jetson Orin Nano
Displays IP address and system information on 128x32 OLED display
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from .oled_driver import OLEDDriver
from .ip_utils import IPDetector


class OLEDDisplayNode(Node):
    """ROS2 node for controlling OLED display"""
    
    def __init__(self):
        super().__init__('oled_display_node')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('update_rate', 5.0)  # Hz
        self.declare_parameter('ip_refresh_interval', 30.0)  # seconds
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.update_rate = self.get_parameter('update_rate').value
        self.ip_refresh_interval = self.get_parameter('ip_refresh_interval').value
        
        # Initialize components
        try:
            self.oled = OLEDDriver(
                i2c_bus=self.i2c_bus,
                i2c_address=self.i2c_address
            )
            self.get_logger().info("OLED driver initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OLED driver: {e}")
            raise
        
        self.ip_detector = IPDetector()
        
        # State variables
        self.current_ip = None
        self.last_ip_check = 0
        self.display_lines = ["Initializing...", "Please wait"]
        self.is_running = True
        
        # Create publisher for IP address
        self.ip_publisher = self.create_publisher(String, 'robot_ip_address', 10)
        
        # Create subscriber for display messages
        self.display_subscriber = self.create_subscription(
            String,
            'oled_display_message',
            self.display_message_callback,
            10
        )
        
        # Create timer for display updates
        self.display_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_display
        )
        
        # Create timer for IP refresh
        self.ip_timer = self.create_timer(
            self.ip_refresh_interval,
            self.refresh_ip_info
        )
        
        # Initial IP detection
        self.refresh_ip_info()
        
        self.get_logger().info(f"OLED Display Node started with update rate: {self.update_rate} Hz")
        
    def refresh_ip_info(self):
        """Refresh IP address information"""
        try:
            self.current_ip = self.ip_detector.get_best_ip()

            if self.current_ip:
                # Display only IP address
                self.display_lines = [f"IP: {self.current_ip}"]

                # Publish IP address
                ip_msg = String()
                ip_msg.data = self.current_ip
                self.ip_publisher.publish(ip_msg)

                self.get_logger().info(f"IP updated: {self.current_ip}")
            else:
                self.display_lines = ["No Network"]
                self.get_logger().warning("No IP address found")

        except Exception as e:
            self.get_logger().error(f"Failed to refresh IP info: {e}")
            self.display_lines = ["IP Error"]
    
    def display_message_callback(self, msg):
        """Handle incoming display messages"""
        try:
            message = msg.data
            # Split message into lines for display
            lines = message.split('\n')
            self.display_lines = [line[:16] for line in lines[:4]]  # Max 4 lines, 16 chars each
            self.get_logger().info(f"Display message received: {message}")
        except Exception as e:
            self.get_logger().error(f"Error processing display message: {e}")
    
    def update_display(self):
        """Update the OLED display"""
        try:
            # Clear display
            self.oled.clear()

            # Display content
            y_offset = 10  # Center vertically for single line
            for line in self.display_lines:
                self.oled.text(line, 0, y_offset)
                y_offset += 10

            # Update display
            self.oled.display()

        except Exception as e:
            self.get_logger().error(f"Failed to update display: {e}")
    

    def shutdown(self):
        """Shutdown the node and cleanup resources"""
        self.get_logger().info("Shutting down OLED display node...")
        self.is_running = False
        
        try:
            # Clear display and show shutdown message
            self.oled.clear()
            self.oled.text("Shutting down...", 0, 10)
            self.oled.display()
            time.sleep(1)
            
            # Close OLED driver
            self.oled.close()
            self.get_logger().info("OLED display node shutdown complete")
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")


def main(args=None):
    """Main function to run the OLED display node"""
    rclpy.init(args=args)
    
    try:
        node = OLEDDisplayNode()
        
        # Handle shutdown gracefully
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.shutdown()
            
    except Exception as e:
        print(f"Failed to start OLED display node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
