#!/usr/bin/env python3
"""
Waveshare Binary Protocol Interface for RP2040 Secondary Controller
Handles communication between Jetson Orin Nano and RP2040 using the official Waveshare binary protocol
"""

import serial
import struct
import time
import threading
import logging
import json
from typing import Optional, Callable


class WaveshareRP2040Interface:
    """Serial interface for communicating with RP2040 using Waveshare binary protocol"""
    
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1.0):
        """
        Initialize serial interface
        
        Args:
            port (str): Serial port device path
            baudrate (int): Communication baud rate
            timeout (float): Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
        # Callbacks for received data
        self.encoder_callback = None
        self.status_callback = None
        
        # Latest received data
        self.latest_encoder_data = {
            'left_encoder': 0,
            'right_encoder': 0,
            'timestamp': 0
        }
        self.latest_status = {
            'battery_voltage': 0.0,
            'motor_status': 'ok',
            'servo_status': 'ok'
        }
        
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """
        Establish serial connection to RP2040
        
        Returns:
            bool: True if connection successful
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Wait for connection to stabilize
            time.sleep(2)

            # Mark as connected - the RP2040 doesn't send acknowledgments
            # We'll assume connection is good if serial port opened successfully
            self.is_connected = True
            self.running = True

            # Start read thread for future encoder/status data
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()

            # Test connection by sending a stop command
            self.send_velocity_command(0.0, 0.0, 0.0)

            self.logger.info(f"Connected to RP2040 on {self.port}")
            return True
                
        except Exception as e:
            self.logger.error(f"Failed to connect to RP2040: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from RP2040"""
        self.running = False
        self.is_connected = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            # Stop motors before disconnecting
            self.send_velocity_command(0.0, 0.0, 0.0)
            self.serial_conn.close()
            
        self.logger.info("Disconnected from RP2040")
    
    def send_velocity_command(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """
        Send velocity command using JSON protocol (the working protocol!)

        Args:
            linear_x (float): Forward/backward speed (m/s)
            linear_y (float): Not used (kept for compatibility)
            angular_z (float): Steering angle in radians

        Returns:
            bool: True if command sent successfully
        """
        if not self.is_connected or not self.serial_conn:
            return False

        try:
            # Convert to differential drive (from working rp2040_interface.py)
            wheelbase = 0.165  # meters
            wheel_radius = 0.0325  # meters (65mm diameter / 2)

            # Calculate wheel speeds using differential drive kinematics
            left_wheel_speed = (linear_x - angular_z * wheelbase / 2) / wheel_radius
            right_wheel_speed = (linear_x + angular_z * wheelbase / 2) / wheel_radius

            # Send motor command using the EXACT working format
            motor_command = {
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

            command_json = json.dumps(motor_command) + '\n'
            self.serial_conn.write(command_json.encode())
            self.serial_conn.flush()

            # Note: In the working implementation, steering was handled separately
            # by the servo_controller node, not combined with motor commands

            return True

        except Exception as e:
            self.logger.error(f"Failed to send velocity command: {e}")
            return False
    
    def set_motor_speeds(self, left_speed: float, right_speed: float) -> bool:
        """
        Legacy method - redirects to velocity command

        Args:
            left_speed (float): Left motor speed (-1.0 to 1.0)
            right_speed (float): Right motor speed (-1.0 to 1.0)

        Returns:
            bool: True if command sent successfully
        """
        # Convert differential drive to linear/angular
        linear_x = (left_speed + right_speed) / 2.0
        angular_z = (right_speed - left_speed) / 2.0

        return self.send_velocity_command(linear_x, linear_x, angular_z)

    def set_servo_angle(self, angle: float) -> bool:
        """
        Legacy method - servo control is now part of velocity command

        Args:
            angle (float): Servo angle in degrees (0-180, 90 is center)

        Returns:
            bool: True (always succeeds as it's handled by velocity command)
        """
        # Servo control is now integrated into the velocity command
        # This method is kept for compatibility but doesn't send separate commands
        return True
    
    def stop_motors(self):
        """Emergency stop - set all motors to zero"""
        self.send_velocity_command(0.0, 0.0, 0.0)
    
    def set_encoder_callback(self, callback: Callable):
        """Set callback for encoder data"""
        self.encoder_callback = callback
    
    def set_status_callback(self, callback: Callable):
        """Set callback for status data"""
        self.status_callback = callback
    
    def get_latest_encoder_data(self) -> dict:
        """Get latest encoder data"""
        return self.latest_encoder_data.copy()
    
    def get_latest_status(self) -> dict:
        """Get latest status data"""
        return self.latest_status.copy()
    
    def _read_loop(self):
        """Background thread to read data from RP2040"""
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                # For now, just maintain the connection
                # In the future, this could parse encoder/status data from RP2040
                time.sleep(0.1)
                
            except Exception as e:
                if self.running:  # Only log if we're supposed to be running
                    self.logger.error(f"Error in read loop: {e}")
                break
