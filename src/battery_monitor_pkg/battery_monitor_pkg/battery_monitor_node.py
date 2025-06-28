#!/usr/bin/env python3
"""
Battery Monitor ROS2 Node for Waveshare JetRacer
Monitors INA219 sensor and publishes battery state information.

CRITICAL: Uses I2C Bus 7 (safe external bus) - NEVER use internal buses on Jetson Orin!
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import BatteryState
from jetracer_msgs.msg import BatteryState as JetracerBatteryState
import logging
import time
from typing import Dict, Optional

from .ina219_driver import INA219Driver


class BatteryMonitorNode(Node):
    """ROS2 node for battery monitoring using INA219 sensor"""
    
    def __init__(self):
        super().__init__('battery_monitor_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('i2c_bus', 7),                    # Safe external I2C bus
                ('i2c_address', 0x41),             # INA219 actual address (found via scan)
                ('publish_rate', 1.0),             # Hz
                ('shunt_ohms', 0.05),              # Shunt resistor value (measured)
                ('max_expected_amps', 3.2),        # Maximum expected current
                ('nominal_voltage', 11.1),         # Battery nominal voltage
                ('max_voltage', 12.6),             # Battery max voltage
                ('min_voltage', 9.0),              # Battery min voltage
                ('capacity', 7.8),                 # Battery capacity in Ah
                ('low_battery_threshold', 20.0),   # Low battery warning %
                ('critical_battery_threshold', 10.0), # Critical battery %
                ('voltage_filter_alpha', 0.9),     # Voltage filtering
                ('current_filter_alpha', 0.8),     # Current filtering
            ]
        )
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.shunt_ohms = self.get_parameter('shunt_ohms').value
        self.max_expected_amps = self.get_parameter('max_expected_amps').value
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.voltage_filter_alpha = self.get_parameter('voltage_filter_alpha').value
        self.current_filter_alpha = self.get_parameter('current_filter_alpha').value
        
        # Battery configuration
        self.battery_config = {
            'nominal_voltage': self.get_parameter('nominal_voltage').value,
            'max_voltage': self.get_parameter('max_voltage').value,
            'min_voltage': self.get_parameter('min_voltage').value,
            'capacity': self.get_parameter('capacity').value,
            'cells': 3,
            'chemistry': 'Li-ion'
        }
        
        # Initialize INA219 driver
        self.ina219_driver = INA219Driver(
            i2c_bus=self.i2c_bus,
            i2c_address=self.i2c_address,
            shunt_ohms=self.shunt_ohms,
            max_expected_amps=self.max_expected_amps,
            battery_config=self.battery_config
        )
        
        # Publishers
        self.battery_state_pub = self.create_publisher(
            BatteryState, 'battery_state', 10)
        self.jetracer_battery_pub = self.create_publisher(
            JetracerBatteryState, 'jetracer/battery_state', 10)
        self.voltage_pub = self.create_publisher(
            Float32, 'battery/voltage', 10)
        self.current_pub = self.create_publisher(
            Float32, 'battery/current', 10)
        self.power_pub = self.create_publisher(
            Float32, 'battery/power', 10)
        self.percentage_pub = self.create_publisher(
            Float32, 'battery/percentage', 10)
        self.status_pub = self.create_publisher(
            String, 'battery/status', 10)
        self.low_battery_pub = self.create_publisher(
            Bool, 'battery/low_battery_warning', 10)
        self.critical_battery_pub = self.create_publisher(
            Bool, 'battery/critical_battery_warning', 10)
        
        # State variables
        self.filtered_voltage = None
        self.filtered_current = None
        self.last_measurement_time = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Timer for periodic measurements
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Log initialization
        if self.ina219_driver.is_initialized:
            self.get_logger().info(f"✅ Battery Monitor initialized successfully")
            self.get_logger().info(f"📍 I2C Bus: {self.i2c_bus}, Address: 0x{self.i2c_address:02X}")
            self.get_logger().info(f"🔋 Battery: {self.battery_config['nominal_voltage']}V, {self.battery_config['capacity']}Ah")
        else:
            self.get_logger().error(f"❌ Failed to initialize INA219: {self.ina219_driver.last_error}")
    
    def timer_callback(self):
        """Timer callback to read and publish battery data"""
        try:
            # Read measurements from INA219
            measurements = self.ina219_driver.read_measurements()
            
            if measurements is None:
                self.consecutive_errors += 1
                if self.consecutive_errors >= self.max_consecutive_errors:
                    self.get_logger().error(f"❌ Too many consecutive errors ({self.consecutive_errors}), attempting sensor reset")
                    if self.ina219_driver.reset():
                        self.get_logger().info("✅ Sensor reset successful")
                        self.consecutive_errors = 0
                    else:
                        self.get_logger().error("❌ Sensor reset failed")
                return
            
            # Reset error counter on successful read
            self.consecutive_errors = 0
            
            # Apply filtering
            self._apply_filtering(measurements)
            
            # Publish all topics
            self._publish_battery_state(measurements)
            self._publish_individual_topics(measurements)
            self._publish_warnings(measurements)
            
            # Update last measurement time
            self.last_measurement_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
            self.consecutive_errors += 1
    
    def _apply_filtering(self, measurements: Dict):
        """Apply exponential filtering to voltage and current"""
        voltage = measurements['supply_voltage']
        current = measurements['current']
        
        if self.filtered_voltage is None:
            self.filtered_voltage = voltage
            self.filtered_current = current
        else:
            self.filtered_voltage = (self.voltage_filter_alpha * self.filtered_voltage + 
                                   (1 - self.voltage_filter_alpha) * voltage)
            self.filtered_current = (self.current_filter_alpha * self.filtered_current + 
                                   (1 - self.current_filter_alpha) * current)
        
        # Update measurements with filtered values
        measurements['filtered_voltage'] = self.filtered_voltage
        measurements['filtered_current'] = self.filtered_current
    
    def _publish_battery_state(self, measurements: Dict):
        """Publish standard ROS2 BatteryState message"""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery"
        
        msg.voltage = measurements['filtered_voltage']
        msg.current = measurements['filtered_current']
        msg.charge = float('nan')  # Not directly measurable
        msg.capacity = self.battery_config['capacity']
        msg.design_capacity = self.battery_config['capacity']
        msg.percentage = measurements['battery_percentage'] / 100.0
        msg.power_supply_status = self._get_power_supply_status(measurements['charging_state'])
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        
        self.battery_state_pub.publish(msg)
    
    def _publish_individual_topics(self, measurements: Dict):
        """Publish individual measurement topics"""
        # Voltage
        voltage_msg = Float32()
        voltage_msg.data = measurements['filtered_voltage']
        self.voltage_pub.publish(voltage_msg)
        
        # Current
        current_msg = Float32()
        current_msg.data = measurements['filtered_current']
        self.current_pub.publish(current_msg)
        
        # Power
        power_msg = Float32()
        power_msg.data = measurements['power']
        self.power_pub.publish(power_msg)
        
        # Percentage
        percentage_msg = Float32()
        percentage_msg.data = measurements['battery_percentage']
        self.percentage_pub.publish(percentage_msg)
        
        # Status
        status_msg = String()
        status_msg.data = f"Battery: {measurements['battery_percentage']:.1f}%, " \
                         f"Voltage: {measurements['filtered_voltage']:.2f}V, " \
                         f"Current: {measurements['filtered_current']:.3f}A, " \
                         f"State: {measurements['charging_state']}"
        self.status_pub.publish(status_msg)
        
        # JetRacer custom battery message
        jetracer_msg = JetracerBatteryState()
        jetracer_msg.header.stamp = self.get_clock().now().to_msg()
        jetracer_msg.header.frame_id = "battery"
        jetracer_msg.battery_name = "main_battery"
        jetracer_msg.voltage = measurements['filtered_voltage']
        jetracer_msg.current = measurements['filtered_current']
        jetracer_msg.power = measurements['power']
        jetracer_msg.capacity = self.battery_config['capacity']
        jetracer_msg.remaining_capacity = (measurements['battery_percentage'] / 100.0) * self.battery_config['capacity']
        jetracer_msg.percentage = measurements['battery_percentage']
        jetracer_msg.temperature = 25.0  # Default temperature (not measured by INA219)
        
        self.jetracer_battery_pub.publish(jetracer_msg)
    
    def _publish_warnings(self, measurements: Dict):
        """Publish battery warning messages"""
        percentage = measurements['battery_percentage']
        
        # Low battery warning
        low_battery_msg = Bool()
        low_battery_msg.data = percentage <= self.low_battery_threshold
        self.low_battery_pub.publish(low_battery_msg)
        
        # Critical battery warning
        critical_battery_msg = Bool()
        critical_battery_msg.data = percentage <= self.critical_battery_threshold
        self.critical_battery_pub.publish(critical_battery_msg)
        
        # Log warnings
        if percentage <= self.critical_battery_threshold:
            self.get_logger().warn(f"🚨 CRITICAL BATTERY: {percentage:.1f}% - IMMEDIATE CHARGING REQUIRED!")
        elif percentage <= self.low_battery_threshold:
            self.get_logger().warn(f"⚠️ LOW BATTERY: {percentage:.1f}% - Please charge soon")
    
    def _get_power_supply_status(self, charging_state: str) -> int:
        """Convert charging state to ROS2 BatteryState power supply status"""
        if charging_state == "charging":
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif charging_state == "discharging":
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING


def main(args=None):
    rclpy.init(args=args)
    
    # Set up logging
    logging.basicConfig(level=logging.INFO)
    
    try:
        battery_monitor = BatteryMonitorNode()
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'battery_monitor' in locals():
            battery_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
