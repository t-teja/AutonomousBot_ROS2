# Servo state message
std_msgs/Header header

# Servo identification
string servo_name

# Current state
float64 current_angle     # Current angle in degrees
float64 target_angle      # Target angle in degrees
float64 angular_velocity  # Current angular velocity in deg/s

# Status information
bool is_moving            # Servo is currently moving
bool is_enabled           # Servo is enabled
bool has_error            # Error condition present
string error_message      # Error description if any

# Physical parameters
float64 min_angle         # Minimum angle limit
float64 max_angle         # Maximum angle limit
float64 max_speed         # Maximum speed capability

# Control information
float64 pwm_value         # Current PWM value (microseconds)
float64 command_timestamp # Timestamp of last command
