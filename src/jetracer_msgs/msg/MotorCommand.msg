# Motor command message
std_msgs/Header header

# Motor identification
string motor_name

# Command type
uint8 VELOCITY_CONTROL = 0
uint8 POSITION_CONTROL = 1
uint8 EFFORT_CONTROL = 2
uint8 PWM_CONTROL = 3

uint8 control_mode

# Command values
float64 velocity          # Target velocity in rad/s
float64 position          # Target position in radians
float64 effort            # Target effort/torque
float64 pwm_duty_cycle    # PWM duty cycle (0.0 to 1.0)

# Control parameters
float64 max_velocity      # Maximum allowed velocity
float64 max_acceleration  # Maximum acceleration
float64 timeout           # Command timeout in seconds

# Enable/disable
bool enable               # Enable motor
bool emergency_stop       # Emergency stop flag
