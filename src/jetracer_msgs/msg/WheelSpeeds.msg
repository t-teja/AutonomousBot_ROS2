# Wheel speeds message for differential drive
std_msgs/Header header

# Individual wheel speeds in rad/s
float64 left_wheel_speed
float64 right_wheel_speed

# Commanded speeds
float64 left_wheel_command
float64 right_wheel_command

# Speed errors (command - actual)
float64 left_wheel_error
float64 right_wheel_error

# Wheel positions in radians
float64 left_wheel_position
float64 right_wheel_position

# Encoder ticks
int64 left_encoder_ticks
int64 right_encoder_ticks

# Timestamps for velocity calculation
builtin_interfaces/Time left_encoder_timestamp
builtin_interfaces/Time right_encoder_timestamp
