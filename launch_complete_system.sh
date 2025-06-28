#!/bin/bash

# JetRacer Complete System Launcher
# Launches all robot nodes using the comprehensive launch file

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Main startup sequence
echo ""
echo -e "${CYAN}🤖 JetRacer Complete System Launcher${NC}"
echo "=================================================="
echo -e "${BLUE}📋 Components: Motor Control + OLED + Battery + Lidar + Web Interface${NC}"
echo -e "${PURPLE}🌐 Web Interface: http://localhost:8080${NC}"
echo ""

# Change to workspace directory
cd /home/orin/Documents/jetros/Autonomous_robot

# Source the workspace
print_status "Sourcing ROS2 workspace..."
source install/setup.bash

# Check if devices exist
print_status "Checking hardware connections..."
if [ -e "/dev/ttyACM0" ]; then
    echo "  ✅ RP2040 controller detected on /dev/ttyACM0"
else
    print_warning "  ⚠️  RP2040 controller not detected on /dev/ttyACM0"
fi

if [ -e "/dev/ttyACM1" ]; then
    echo "  ✅ RPLidar detected on /dev/ttyACM1"
else
    print_warning "  ⚠️  RPLidar not detected on /dev/ttyACM1"
fi

echo ""
print_status "Starting complete JetRacer system..."
echo ""

# Launch the complete system with explicit lidar parameter
print_status "Launching complete system with lidar enabled..."
ros2 launch web_interface_pkg jetracer_complete_system.launch.py use_lidar:=true use_rviz:=false

echo ""
echo "✅ JetRacer System Stopped!"
echo "=================================================="
