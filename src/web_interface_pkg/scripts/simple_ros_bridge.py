#!/usr/bin/env python3
"""
Simple ROS2 Bridge for Web Interface
Simplified version to get battery data working
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import time
import threading
import asyncio
import websockets
from typing import Set


class SimpleRosBridge(Node):
    """Simple ROS2 bridge for web interface"""
    
    def __init__(self):
        super().__init__('simple_ros_bridge')
        
        # WebSocket settings
        self.websocket_host = '0.0.0.0'
        self.websocket_port = 8765
        self.connected_clients: Set = set()
        
        # Data storage
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
        
        # Timer for broadcasting data
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_data)  # 2Hz
        
        # Start WebSocket server
        self.start_websocket_server()
        
        self.get_logger().info("Simple ROS Bridge initialized")
        self.get_logger().info(f"WebSocket server on {self.websocket_host}:{self.websocket_port}")
    
    def voltage_callback(self, msg):
        """Handle battery voltage"""
        self.battery_data['voltage'] = msg.data
        self.battery_data['timestamp'] = time.time()
        self.get_logger().debug(f"Battery voltage: {msg.data:.2f}V")
    
    def percentage_callback(self, msg):
        """Handle battery percentage"""
        self.battery_data['percentage'] = msg.data
        self.battery_data['timestamp'] = time.time()
        self.get_logger().debug(f"Battery percentage: {msg.data:.1f}%")
    
    def start_websocket_server(self):
        """Start WebSocket server in separate thread"""
        def run_server():
            async def handler(websocket, path):
                self.connected_clients.add(websocket)
                self.get_logger().info(f"Client connected from {websocket.remote_address}")
                try:
                    await websocket.wait_closed()
                except Exception as e:
                    self.get_logger().debug(f"Client disconnected: {e}")
                finally:
                    self.connected_clients.discard(websocket)
                    self.get_logger().info("Client disconnected")

            async def start_server():
                server = await websockets.serve(handler, self.websocket_host, self.websocket_port)
                self.get_logger().info(f"WebSocket server started on {self.websocket_host}:{self.websocket_port}")
                await server.wait_closed()

            try:
                asyncio.run(start_server())
            except Exception as e:
                self.get_logger().error(f"WebSocket server error: {e}")

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    def broadcast_data(self):
        """Broadcast battery data to connected clients"""
        if not self.connected_clients:
            return
        
        # Prepare data package
        data_package = {
            'type': 'robot_data',
            'data': {
                'battery': {
                    'voltage_raw': self.battery_data['voltage'],
                    'percentage_raw': self.battery_data['percentage'],
                    'timestamp': self.battery_data['timestamp']
                },
                'robot_status': {
                    'status': 'Simple Bridge Active',
                    'ip_address': '192.168.1.100',  # You can get real IP later
                    'timestamp': time.time()
                }
            },
            'timestamp': time.time()
        }
        
        message = json.dumps(data_package)
        
        # Send to all clients (simplified approach)
        disconnected = set()
        for client in list(self.connected_clients):
            try:
                # Use a simple approach - create a new event loop for sending
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(client.send(message))
                loop.close()
            except Exception as e:
                self.get_logger().debug(f"Failed to send to client: {e}")
                disconnected.add(client)

        # Remove disconnected clients
        self.connected_clients -= disconnected
        
        if self.connected_clients:
            self.get_logger().debug(f"Sent data to {len(self.connected_clients)} clients")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = SimpleRosBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
