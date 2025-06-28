#!/usr/bin/env python3
"""
ROS2 Bridge Node for Web Interface
Bridges ROS2 topics with WebSocket for real-time web communication
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import asyncio
import websockets
import threading
import time
from typing import Dict, Any, Set

# ROS2 Message Imports
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from jetracer_msgs.msg import BatteryState as JetracerBatteryState


class RosBridgeNode(Node):
    """ROS2 node that bridges topics with WebSocket for web interface"""
    
    def __init__(self):
        super().__init__('ros_bridge_node')
        
        # WebSocket server settings
        self.websocket_host = '0.0.0.0'
        self.websocket_port = 8765
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        
        # Data storage for latest values
        self.latest_data = {
            'battery': {},
            'robot_status': {},
            'lidar': {},
            'odometry': {},
            'system': {}
        }
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Initialize ROS2 subscribers
        self.setup_subscribers()
        
        # Initialize ROS2 publishers
        self.setup_publishers()
        
        # Start WebSocket server in separate thread
        self.websocket_thread = threading.Thread(target=self.start_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        # Timer for periodic data broadcast
        self.broadcast_timer = self.create_timer(0.1, self.broadcast_data)  # 10Hz
        
        self.get_logger().info("ROS Bridge Node initialized")
        self.get_logger().info(f"WebSocket server starting on {self.websocket_host}:{self.websocket_port}")
    
    def setup_subscribers(self):
        """Setup ROS2 subscribers for all robot data"""
        
        # Battery monitoring
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, self.reliable_qos)
        
        self.jetracer_battery_sub = self.create_subscription(
            JetracerBatteryState, '/jetracer/battery_state', self.jetracer_battery_callback, self.reliable_qos)
        
        self.battery_voltage_sub = self.create_subscription(
            Float32, '/battery/voltage', self.battery_voltage_callback, self.reliable_qos)
        
        self.battery_percentage_sub = self.create_subscription(
            Float32, '/battery/percentage', self.battery_percentage_callback, self.reliable_qos)
        
        # Robot status
        self.robot_status_sub = self.create_subscription(
            String, '/robot_status', self.robot_status_callback, self.reliable_qos)
        
        self.robot_ip_sub = self.create_subscription(
            String, '/robot_ip_address', self.robot_ip_callback, self.reliable_qos)
        
        # Lidar data
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, self.sensor_qos)
        
        self.lidar_health_sub = self.create_subscription(
            Bool, '/lidar/health_status', self.lidar_health_callback, self.reliable_qos)
        
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, self.sensor_qos)
    
    def setup_publishers(self):
        """Setup ROS2 publishers for robot control"""
        
        # Robot movement control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', self.reliable_qos)
        
        # OLED display control
        self.oled_msg_pub = self.create_publisher(String, '/oled_display_message', self.reliable_qos)
    
    def battery_callback(self, msg):
        """Handle battery state messages"""
        self.latest_data['battery'].update({
            'voltage': msg.voltage,
            'current': msg.current,
            'charge': msg.charge,
            'percentage': msg.percentage,
            'power_supply_status': msg.power_supply_status,
            'power_supply_health': msg.power_supply_health,
            'timestamp': time.time()
        })
    
    def jetracer_battery_callback(self, msg):
        """Handle JetRacer custom battery messages"""
        self.latest_data['battery'].update({
            'jetracer_voltage': msg.voltage,
            'jetracer_current': msg.current,
            'jetracer_power': msg.power,
            'jetracer_percentage': msg.percentage,
            'jetracer_power_supply_status': msg.power_supply_status,
            'timestamp': time.time()
        })
    
    def battery_voltage_callback(self, msg):
        """Handle battery voltage messages"""
        self.latest_data['battery']['voltage_raw'] = msg.data
    
    def battery_percentage_callback(self, msg):
        """Handle battery percentage messages"""
        self.latest_data['battery']['percentage_raw'] = msg.data
    
    def robot_status_callback(self, msg):
        """Handle robot status messages"""
        self.latest_data['robot_status']['status'] = msg.data
        self.latest_data['robot_status']['timestamp'] = time.time()
    
    def robot_ip_callback(self, msg):
        """Handle robot IP address messages"""
        self.latest_data['robot_status']['ip_address'] = msg.data
    
    def scan_callback(self, msg):
        """Handle laser scan messages"""
        import math

        # Downsample scan data for web transmission
        ranges = []
        for r in msg.ranges[::10]:  # Every 10th point
            # Convert NaN and inf values to None for JSON serialization
            if math.isnan(r) or math.isinf(r):
                ranges.append(None)
            else:
                ranges.append(float(r))

        intensities = []
        if msg.intensities:
            for i in msg.intensities[::10]:
                if math.isnan(i) or math.isinf(i):
                    intensities.append(None)
                else:
                    intensities.append(float(i))

        self.latest_data['lidar'].update({
            'ranges': ranges,
            'intensities': intensities,
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment * 10),  # Adjusted for downsampling
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'timestamp': time.time()
        })
    
    def lidar_health_callback(self, msg):
        """Handle lidar health status"""
        self.latest_data['lidar']['health'] = msg.data
    
    def odom_callback(self, msg):
        """Handle odometry messages"""
        self.latest_data['odometry'].update({
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            },
            'timestamp': time.time()
        })
    
    async def handle_websocket_message(self, websocket, message):
        """Handle incoming WebSocket messages from web interface"""
        try:
            self.get_logger().info(f"Received WebSocket message: {message}")
            data = json.loads(message)
            command = data.get('command')
            self.get_logger().info(f"Parsed command: {command}")

            if command == 'move_robot':
                # Handle robot movement commands
                twist = Twist()
                twist.linear.x = float(data.get('linear_x', 0.0))
                twist.linear.y = float(data.get('linear_y', 0.0))
                twist.angular.z = float(data.get('angular_z', 0.0))
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

            elif command == 'stop_robot':
                # Stop robot movement
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Published stop command")

            elif command == 'send_oled_message':
                # Send message to OLED display
                msg = String()
                msg.data = data.get('message', '')
                self.oled_msg_pub.publish(msg)
                self.get_logger().info(f"Sent OLED message: {msg.data}")

            elif command == 'start_lidar':
                # Start lidar node (would need process management)
                self.get_logger().info("Start lidar command received")

            elif command == 'stop_lidar':
                # Stop lidar node (would need process management)
                self.get_logger().info("Stop lidar command received")

            else:
                self.get_logger().warning(f"Unknown command received: {command}")

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON received: {message}")
        except Exception as e:
            self.get_logger().error(f"Error handling WebSocket message: {e}")
    
    async def websocket_handler(self, websocket):
        """Handle WebSocket connections"""
        self.connected_clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")

        try:
            async for message in websocket:
                await self.handle_websocket_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            self.connected_clients.discard(websocket)
            self.get_logger().info(f"Client disconnected: {websocket.remote_address}")
    
    def start_websocket_server(self):
        """Start WebSocket server in asyncio event loop"""
        try:
            self.websocket_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.websocket_loop)

            async def start_server():
                server = await websockets.serve(
                    self.websocket_handler,
                    self.websocket_host,
                    self.websocket_port
                )
                self.get_logger().info(f"WebSocket server started on {self.websocket_host}:{self.websocket_port}")
                await server.wait_closed()

            self.websocket_loop.run_until_complete(start_server())
        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {e}")

    def clean_data_for_json(self, data):
        """Clean data to ensure JSON serialization compatibility"""
        import math

        if isinstance(data, dict):
            cleaned = {}
            for key, value in data.items():
                cleaned[key] = self.clean_data_for_json(value)
            return cleaned
        elif isinstance(data, list):
            return [self.clean_data_for_json(item) for item in data]
        elif isinstance(data, float):
            if math.isnan(data) or math.isinf(data):
                return None  # Convert NaN/Inf to None (null in JSON)
            return data
        else:
            return data

    def broadcast_data(self):
        """Broadcast latest data to all connected clients"""
        if not self.connected_clients:
            return

        # Clean data to ensure JSON compatibility (remove NaN/Inf values)
        cleaned_data = self.clean_data_for_json(self.latest_data)

        # Prepare data package
        data_package = {
            'type': 'robot_data',
            'data': cleaned_data,
            'timestamp': time.time()
        }

        try:
            # Send to all connected clients
            message = json.dumps(data_package)
            disconnected_clients = set()

            # Log data being sent (every 10 broadcasts to see more detail)
            if hasattr(self, '_broadcast_count'):
                self._broadcast_count += 1
            else:
                self._broadcast_count = 1

            if self._broadcast_count % 10 == 0:
                lidar_ranges = len(self.latest_data.get('lidar', {}).get('ranges', []))
                battery_voltage = self.latest_data.get('battery', {}).get('voltage_raw', 'N/A')
                lidar_data_sample = self.latest_data.get('lidar', {})
                self.get_logger().info(f"Broadcasting data: {len(self.connected_clients)} clients, "
                                     f"lidar points: {lidar_ranges}, battery: {battery_voltage}V")
                self.get_logger().info(f"Lidar data keys: {list(lidar_data_sample.keys())}")
                if 'ranges' in lidar_data_sample and lidar_data_sample['ranges']:
                    self.get_logger().info(f"First 5 ranges: {lidar_data_sample['ranges'][:5]}")

            for client in list(self.connected_clients):
                try:
                    if hasattr(self, 'websocket_loop') and self.websocket_loop:
                        # Create a future to send the message
                        future = asyncio.run_coroutine_threadsafe(
                            client.send(message),
                            self.websocket_loop
                        )
                        # Don't wait for completion to avoid blocking
                except Exception as e:
                    self.get_logger().debug(f"Failed to send to client: {e}")
                    disconnected_clients.add(client)

            # Remove disconnected clients
            self.connected_clients -= disconnected_clients

        except Exception as e:
            self.get_logger().error(f"Error in broadcast_data: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = RosBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in ROS Bridge Node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
