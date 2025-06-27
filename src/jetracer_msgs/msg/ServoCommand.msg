# Servo command message
std_msgs/Header header

# Servo identification
string servo_name

# Command values
float64 angle             # Target angle in degrees
float64 speed             # Servo speed (0-100%)
float64 timeout           # Command timeout in seconds

# Control parameters
float64 min_angle         # Minimum allowed angle
float64 max_angle         # Maximum allowed angle

# Enable/disable
bool enable               # Enable servo
bool emergency_stop       # Emergency stop flag
