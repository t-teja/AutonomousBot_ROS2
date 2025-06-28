#!/usr/bin/env python3
"""
Web Server Node for JetRacer Robot Interface
Serves static web files and handles HTTP requests
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tornado.web
import tornado.ioloop
import tornado.websocket
import os
import json
import time
import threading
from ament_index_python.packages import get_package_share_directory


class WebServerNode(Node):
    """ROS2 node that serves web interface files"""

    def __init__(self):
        super().__init__('web_server_node')

        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')

        # Get parameters
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value

        # Battery data storage
        self.battery_data = {
            'voltage': 0.0,
            'percentage': 0.0,
            'timestamp': time.time()
        }

        # ROS2 subscribers for battery data
        self.voltage_sub = self.create_subscription(
            Float32, '/battery/voltage', self.voltage_callback, 10)

        self.percentage_sub = self.create_subscription(
            Float32, '/battery/percentage', self.percentage_callback, 10)
        
        # Get web directory path
        try:
            package_share_dir = get_package_share_directory('web_interface_pkg')
            self.web_dir = os.path.join(package_share_dir, 'web')
        except Exception:
            # Fallback for development
            self.web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
            self.web_dir = os.path.abspath(self.web_dir)
        
        self.get_logger().info(f"Web directory: {self.web_dir}")
        
        # Setup Tornado application
        self.setup_web_app()
        
        # Start web server in separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f"Web server starting on http://{self.host}:{self.port}")

    def voltage_callback(self, msg):
        """Handle battery voltage updates"""
        self.battery_data['voltage'] = msg.data
        self.battery_data['timestamp'] = time.time()
        self.get_logger().debug(f"Battery voltage: {msg.data:.2f}V")

    def percentage_callback(self, msg):
        """Handle battery percentage updates"""
        self.battery_data['percentage'] = msg.data
        self.battery_data['timestamp'] = time.time()
        self.get_logger().debug(f"Battery percentage: {msg.data:.1f}%")

    def setup_web_app(self):
        """Setup Tornado web application with routes"""
        
        # Define application routes
        handlers = [
            (r"/", MainHandler, {"web_dir": self.web_dir}),
            (r"/api/robot_info", RobotInfoHandler),
            (r"/api/battery_data", BatteryDataHandler, {"web_server_node": self}),
            (r"/api/system_status", SystemStatusHandler),
            (r"/(.*)", StaticFileHandler, {"path": self.web_dir}),
        ]
        
        # Application settings
        settings = {
            "debug": True,
            "static_path": self.web_dir,
            "template_path": self.web_dir,
        }
        
        self.app = tornado.web.Application(handlers, **settings)
    
    def start_server(self):
        """Start the Tornado web server"""
        try:
            self.app.listen(self.port, address=self.host)
            tornado.ioloop.IOLoop.current().start()
        except Exception as e:
            self.get_logger().error(f"Failed to start web server: {e}")


class MainHandler(tornado.web.RequestHandler):
    """Main page handler"""
    
    def initialize(self, web_dir):
        self.web_dir = web_dir
    
    def get(self):
        """Serve main index.html page"""
        index_path = os.path.join(self.web_dir, 'index.html')
        if os.path.exists(index_path):
            with open(index_path, 'r') as f:
                self.write(f.read())
        else:
            self.write("""
            <!DOCTYPE html>
            <html>
            <head>
                <title>JetRacer Robot Interface</title>
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
            </head>
            <body>
                <h1>JetRacer Robot Interface</h1>
                <p>Web interface files not found. Please build the package properly.</p>
                <p>Expected location: {}</p>
            </body>
            </html>
            """.format(index_path))


class RobotInfoHandler(tornado.web.RequestHandler):
    """API handler for robot information"""

    def get(self):
        """Return robot information"""
        robot_info = {
            "name": "JetRacer Autonomous Robot",
            "version": "1.0.0",
            "hardware": {
                "platform": "Jetson Orin Nano",
                "lidar": "RPLidar A1M8",
                "camera": "IMX219",
                "display": "128x32 OLED",
                "battery_monitor": "INA219"
            },
            "capabilities": [
                "Motor Control",
                "Battery Monitoring",
                "OLED Display",
                "Lidar Scanning",
                "Web Interface",
                "Remote Control"
            ],
            "status": "operational"
        }

        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(robot_info, indent=2))


class BatteryDataHandler(tornado.web.RequestHandler):
    """API handler for battery data"""

    def initialize(self, web_server_node):
        self.web_server_node = web_server_node

    def get(self):
        """Return current battery data from stored ROS data"""
        try:
            battery_data = {
                "voltage": self.web_server_node.battery_data['voltage'],
                "percentage": self.web_server_node.battery_data['percentage'],
                "status": "active" if self.web_server_node.battery_data['voltage'] > 0 else "inactive",
                "timestamp": self.web_server_node.battery_data['timestamp']
            }

        except Exception as e:
            battery_data = {
                "voltage": 0.0,
                "percentage": 0.0,
                "status": "error",
                "error": str(e),
                "timestamp": time.time()
            }

        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(battery_data, indent=2))


class SystemStatusHandler(tornado.web.RequestHandler):
    """API handler for system status"""
    
    def get(self):
        """Return system status"""
        import psutil
        import time
        
        try:
            status = {
                "timestamp": time.time(),
                "cpu_percent": psutil.cpu_percent(interval=1),
                "memory": {
                    "total": psutil.virtual_memory().total,
                    "available": psutil.virtual_memory().available,
                    "percent": psutil.virtual_memory().percent
                },
                "disk": {
                    "total": psutil.disk_usage('/').total,
                    "free": psutil.disk_usage('/').free,
                    "percent": psutil.disk_usage('/').percent
                },
                "temperature": self.get_temperature(),
                "uptime": time.time() - psutil.boot_time()
            }
        except Exception as e:
            status = {
                "error": str(e),
                "timestamp": time.time()
            }
        
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(status, indent=2))
    
    def get_temperature(self):
        """Get system temperature"""
        try:
            # Try to read Jetson temperature
            temp_files = [
                '/sys/class/thermal/thermal_zone0/temp',
                '/sys/class/thermal/thermal_zone1/temp'
            ]
            
            for temp_file in temp_files:
                if os.path.exists(temp_file):
                    with open(temp_file, 'r') as f:
                        temp = float(f.read().strip()) / 1000.0  # Convert from millidegrees
                        return temp
            
            return None
        except Exception:
            return None


class StaticFileHandler(tornado.web.StaticFileHandler):
    """Custom static file handler with proper MIME types"""
    
    def get_content_type(self):
        """Set proper content types for web files"""
        if self.path.endswith('.js'):
            return 'application/javascript'
        elif self.path.endswith('.css'):
            return 'text/css'
        elif self.path.endswith('.html'):
            return 'text/html'
        elif self.path.endswith('.json'):
            return 'application/json'
        else:
            return super().get_content_type()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = WebServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Web Server Node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
