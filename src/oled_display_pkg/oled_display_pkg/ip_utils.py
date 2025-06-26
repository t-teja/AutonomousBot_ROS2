#!/usr/bin/env python3
"""
IP Address Detection Utilities for Jetson Orin Nano
Handles multiple network interfaces and provides formatted IP information
"""

import socket
import subprocess
import netifaces
import logging
from typing import Dict, List, Optional


class IPDetector:
    """Utility class for detecting and managing IP addresses"""
    
    def __init__(self):
        """Initialize IP detector"""
        self.logger = logging.getLogger(__name__)
        
    def get_primary_ip(self) -> Optional[str]:
        """
        Get the primary IP address (the one used for internet connectivity)
        
        Returns:
            str: Primary IP address or None if not found
        """
        try:
            # Connect to a remote address to determine the primary interface
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                # Use Google's DNS server to determine route
                s.connect(("8.8.8.8", 80))
                primary_ip = s.getsockname()[0]
                self.logger.info(f"Primary IP detected: {primary_ip}")
                return primary_ip
        except Exception as e:
            self.logger.warning(f"Failed to get primary IP via socket: {e}")
            return None
    
    def get_all_interfaces(self) -> Dict[str, str]:
        """
        Get all network interfaces and their IP addresses
        
        Returns:
            dict: Dictionary mapping interface names to IP addresses
        """
        interfaces = {}
        
        try:
            # Get all network interfaces
            for interface in netifaces.interfaces():
                try:
                    # Skip loopback interface
                    if interface == 'lo':
                        continue
                    
                    # Get IPv4 addresses for this interface
                    addrs = netifaces.ifaddresses(interface)
                    if netifaces.AF_INET in addrs:
                        ipv4_info = addrs[netifaces.AF_INET][0]
                        ip_addr = ipv4_info.get('addr')
                        if ip_addr and not ip_addr.startswith('127.'):
                            interfaces[interface] = ip_addr
                            
                except Exception as e:
                    self.logger.warning(f"Error processing interface {interface}: {e}")
                    continue
                    
        except Exception as e:
            self.logger.error(f"Failed to get network interfaces: {e}")
            
        return interfaces
    
    def get_wifi_ip(self) -> Optional[str]:
        """
        Get WiFi interface IP address
        
        Returns:
            str: WiFi IP address or None if not found
        """
        wifi_interfaces = ['wlan0', 'wlp1s0', 'wlp2s0', 'wifi0']
        interfaces = self.get_all_interfaces()
        
        for wifi_if in wifi_interfaces:
            if wifi_if in interfaces:
                self.logger.info(f"WiFi IP found on {wifi_if}: {interfaces[wifi_if]}")
                return interfaces[wifi_if]
        
        # Check for any interface starting with 'wl'
        for interface, ip in interfaces.items():
            if interface.startswith('wl'):
                self.logger.info(f"WiFi IP found on {interface}: {ip}")
                return ip
                
        return None
    
    def get_ethernet_ip(self) -> Optional[str]:
        """
        Get Ethernet interface IP address
        
        Returns:
            str: Ethernet IP address or None if not found
        """
        eth_interfaces = ['eth0', 'enp1s0', 'eno1', 'enx']
        interfaces = self.get_all_interfaces()
        
        for eth_if in eth_interfaces:
            if eth_if in interfaces:
                self.logger.info(f"Ethernet IP found on {eth_if}: {interfaces[eth_if]}")
                return interfaces[eth_if]
        
        # Check for any interface starting with 'en' or 'eth'
        for interface, ip in interfaces.items():
            if interface.startswith(('en', 'eth')):
                self.logger.info(f"Ethernet IP found on {interface}: {ip}")
                return ip
                
        return None
    
    def get_best_ip(self) -> Optional[str]:
        """
        Get the best available IP address with preference order:
        1. Primary IP (internet-facing)
        2. Ethernet IP
        3. WiFi IP
        4. Any available IP
        
        Returns:
            str: Best available IP address or None if none found
        """
        # Try primary IP first
        primary_ip = self.get_primary_ip()
        if primary_ip:
            return primary_ip
        
        # Try Ethernet
        eth_ip = self.get_ethernet_ip()
        if eth_ip:
            return eth_ip
        
        # Try WiFi
        wifi_ip = self.get_wifi_ip()
        if wifi_ip:
            return wifi_ip
        
        # Try any available interface
        interfaces = self.get_all_interfaces()
        if interfaces:
            interface, ip = next(iter(interfaces.items()))
            self.logger.info(f"Using fallback IP from {interface}: {ip}")
            return ip
        
        self.logger.warning("No IP addresses found")
        return None
    
    def get_hostname(self) -> str:
        """
        Get the system hostname
        
        Returns:
            str: System hostname
        """
        try:
            hostname = socket.gethostname()
            self.logger.info(f"Hostname: {hostname}")
            return hostname
        except Exception as e:
            self.logger.error(f"Failed to get hostname: {e}")
            return "unknown"
    
    def get_network_info(self) -> Dict[str, str]:
        """
        Get comprehensive network information
        
        Returns:
            dict: Network information including hostname, primary IP, and all interfaces
        """
        info = {
            'hostname': self.get_hostname(),
            'primary_ip': self.get_primary_ip(),
            'wifi_ip': self.get_wifi_ip(),
            'ethernet_ip': self.get_ethernet_ip(),
            'best_ip': self.get_best_ip(),
            'all_interfaces': self.get_all_interfaces()
        }
        
        return info
    
    def format_ip_for_display(self, max_length=16) -> List[str]:
        """
        Format IP information for OLED display
        
        Args:
            max_length (int): Maximum characters per line
            
        Returns:
            list: List of formatted strings for display
        """
        lines = []
        
        # Get best IP
        best_ip = self.get_best_ip()
        hostname = self.get_hostname()
        
        if best_ip:
            # Add hostname if it fits
            if len(hostname) <= max_length:
                lines.append(hostname)
            else:
                lines.append(hostname[:max_length-3] + "...")
            
            # Add IP address
            ip_line = f"IP: {best_ip}"
            if len(ip_line) <= max_length:
                lines.append(ip_line)
            else:
                lines.append(best_ip)
        else:
            lines.append("No Network")
            lines.append("Connection")
        
        return lines


if __name__ == "__main__":
    # Test the IP detector
    logging.basicConfig(level=logging.INFO)
    
    detector = IPDetector()
    
    print("Network Information:")
    print("-" * 30)
    
    info = detector.get_network_info()
    for key, value in info.items():
        if key == 'all_interfaces':
            print(f"{key}:")
            for interface, ip in value.items():
                print(f"  {interface}: {ip}")
        else:
            print(f"{key}: {value}")
    
    print("\nFormatted for OLED:")
    print("-" * 20)
    display_lines = detector.format_ip_for_display()
    for i, line in enumerate(display_lines):
        print(f"Line {i+1}: {line}")
