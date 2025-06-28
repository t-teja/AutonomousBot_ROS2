#!/usr/bin/env python3
"""
INA219 Battery Monitor Driver for Waveshare JetRacer
Provides voltage, current, and power measurements with battery percentage calculation.

CRITICAL: Uses I2C Bus 7 (safe external bus) - NEVER use internal buses on Jetson Orin!
"""

import logging
import time
from typing import Dict, Optional, Tuple
from ina219 import INA219
from ina219 import DeviceRangeError


class INA219Driver:
    """Driver for INA219 current/voltage sensor with battery monitoring capabilities"""
    
    def __init__(self,
                 i2c_bus: int = 7,
                 i2c_address: int = 0x41,
                 shunt_ohms: float = 0.05,
                 max_expected_amps: float = 3.2,
                 battery_config: Optional[Dict] = None):
        """
        Initialize INA219 driver
        
        Args:
            i2c_bus: I2C bus number (7 for external devices on Jetson Orin)
            i2c_address: INA219 I2C address (default 0x40)
            shunt_ohms: Shunt resistor value in ohms (default 0.1)
            max_expected_amps: Maximum expected current in amperes
            battery_config: Battery configuration dictionary
        """
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.shunt_ohms = shunt_ohms
        self.max_expected_amps = max_expected_amps
        
        # Default battery configuration for 3x 18650 Li-ion cells
        self.battery_config = battery_config or {
            'nominal_voltage': 11.1,    # V (3.7V x 3)
            'max_voltage': 12.6,        # V (4.2V x 3) 
            'min_voltage': 9.0,         # V (3.0V x 3)
            'capacity': 7.8,            # Ah (2600mAh x 3)
            'cells': 3,
            'chemistry': 'Li-ion'
        }
        
        self.ina = None
        self.is_initialized = False
        self.last_error = None
        
        # Initialize the sensor
        self._initialize_sensor()
        
    def _initialize_sensor(self) -> bool:
        """Initialize the INA219 sensor"""
        try:
            # Create INA219 instance with custom I2C bus
            self.ina = INA219(self.shunt_ohms, self.max_expected_amps, 
                            address=self.i2c_address, busnum=self.i2c_bus)
            
            # Configure the sensor
            self.ina.configure(voltage_range=self.ina.RANGE_16V,
                             gain=self.ina.GAIN_AUTO,
                             bus_adc=self.ina.ADC_128SAMP,
                             shunt_adc=self.ina.ADC_128SAMP)
            
            # Test read to verify connection
            _ = self.ina.voltage()
            
            self.is_initialized = True
            self.last_error = None
            logging.info(f"INA219 initialized successfully on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02X}")
            return True
            
        except Exception as e:
            self.is_initialized = False
            self.last_error = str(e)
            logging.error(f"Failed to initialize INA219: {e}")
            return False
    
    def read_measurements(self) -> Optional[Dict]:
        """
        Read voltage, current, and power measurements
        
        Returns:
            Dictionary with measurement data or None if error
        """
        if not self.is_initialized:
            logging.warning("INA219 not initialized, attempting to reinitialize...")
            if not self._initialize_sensor():
                return None
        
        try:
            # Read raw measurements
            bus_voltage = self.ina.voltage()  # Voltage on V- (load side)
            shunt_voltage = self.ina.shunt_voltage() / 1000.0  # Convert mV to V
            current = self.ina.current() / 1000.0  # Convert mA to A
            power = self.ina.power() / 1000.0  # Convert mW to W
            
            # Calculate supply voltage (battery voltage)
            supply_voltage = bus_voltage + shunt_voltage
            
            # Calculate battery percentage
            battery_percentage = self._calculate_battery_percentage(supply_voltage)
            
            # Determine charging state
            charging_state = self._determine_charging_state(current)
            
            measurements = {
                'timestamp': time.time(),
                'bus_voltage': bus_voltage,           # V (load side voltage)
                'supply_voltage': supply_voltage,     # V (battery voltage)
                'shunt_voltage': shunt_voltage,       # V (voltage across shunt)
                'current': current,                   # A (positive = discharging)
                'power': power,                       # W
                'battery_percentage': battery_percentage,  # %
                'charging_state': charging_state,
                'status': 'healthy'
            }
            
            self.last_error = None
            return measurements
            
        except DeviceRangeError as e:
            logging.error(f"INA219 device range error: {e}")
            self.last_error = f"Device range error: {e}"
            return None
            
        except Exception as e:
            logging.error(f"Error reading INA219 measurements: {e}")
            self.last_error = str(e)
            return None
    
    def _calculate_battery_percentage(self, voltage: float) -> float:
        """
        Calculate battery percentage based on voltage
        
        Args:
            voltage: Battery voltage in volts
            
        Returns:
            Battery percentage (0-100)
        """
        min_v = self.battery_config['min_voltage']
        max_v = self.battery_config['max_voltage']
        
        # Clamp voltage to valid range
        voltage = max(min_v, min(max_v, voltage))
        
        # Linear interpolation for Li-ion batteries (simplified)
        # Note: Real Li-ion discharge curves are non-linear
        percentage = ((voltage - min_v) / (max_v - min_v)) * 100.0
        
        return round(percentage, 1)
    
    def _determine_charging_state(self, current: float) -> str:
        """
        Determine charging state based on current flow

        Args:
            current: Current in amperes (positive = charging for this setup)

        Returns:
            Charging state string
        """
        if current > 0.1:  # Charging threshold (positive current = charging)
            return "charging"
        elif current < -0.1:  # Discharging threshold (negative current = discharging)
            return "discharging"
        else:
            return "idle"
    
    def get_battery_info(self) -> Dict:
        """Get battery configuration information"""
        return {
            'battery_config': self.battery_config.copy(),
            'sensor_config': {
                'i2c_bus': self.i2c_bus,
                'i2c_address': f"0x{self.i2c_address:02X}",
                'shunt_ohms': self.shunt_ohms,
                'max_expected_amps': self.max_expected_amps
            },
            'is_initialized': self.is_initialized,
            'last_error': self.last_error
        }
    
    def reset(self) -> bool:
        """Reset and reinitialize the sensor"""
        self.is_initialized = False
        return self._initialize_sensor()


if __name__ == "__main__":
    # Test the driver
    logging.basicConfig(level=logging.INFO)
    
    print("Testing INA219 Driver...")
    driver = INA219Driver()
    
    if driver.is_initialized:
        print("✅ INA219 initialized successfully")
        
        # Read measurements
        data = driver.read_measurements()
        if data:
            print(f"🔋 Battery Voltage: {data['supply_voltage']:.2f}V")
            print(f"⚡ Current: {data['current']:.3f}A")
            print(f"🔌 Power: {data['power']:.2f}W")
            print(f"📊 Battery: {data['battery_percentage']:.1f}%")
            print(f"🔄 State: {data['charging_state']}")
        else:
            print("❌ Failed to read measurements")
    else:
        print(f"❌ Failed to initialize INA219: {driver.last_error}")
