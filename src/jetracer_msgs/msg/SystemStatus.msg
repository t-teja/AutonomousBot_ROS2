# System status message
std_msgs/Header header

# System identification
string robot_name
string hardware_version
string software_version

# Overall system state
uint8 SYSTEM_OK = 0
uint8 SYSTEM_WARNING = 1
uint8 SYSTEM_ERROR = 2
uint8 SYSTEM_EMERGENCY = 3

uint8 system_state
string status_message

# Component status
bool rp2040_connected
bool imu_available
bool lidar_available
bool camera_available
bool battery_ok

# Performance metrics
float64 cpu_usage         # CPU usage percentage
float64 memory_usage      # Memory usage percentage
float64 disk_usage        # Disk usage percentage
float64 temperature       # System temperature in Celsius

# Network status
bool wifi_connected
string ip_address
float64 network_latency   # Network latency in ms

# Uptime
builtin_interfaces/Duration uptime

# Error counts
uint32 total_errors
uint32 communication_errors
uint32 sensor_errors
uint32 actuator_errors
