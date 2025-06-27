# Motor state information
std_msgs/Header header

# Motor identification
string motor_name

# Current state
float64 velocity          # Current velocity in rad/s
float64 position          # Current position in radians
float64 effort            # Current effort/torque
float64 current           # Motor current in Amperes
float64 temperature       # Motor temperature in Celsius

# Encoder information
int64 encoder_ticks       # Raw encoder ticks
float64 encoder_velocity  # Encoder-based velocity

# Control information
float64 command_velocity  # Commanded velocity
float64 pid_output        # PID controller output
float64 pwm_duty_cycle    # PWM duty cycle (0.0 to 1.0)

# Status flags
bool is_enabled           # Motor is enabled
bool has_error            # Error condition present
string error_message      # Error description if any
