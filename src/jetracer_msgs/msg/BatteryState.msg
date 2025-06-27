# Battery state message
std_msgs/Header header

# Battery identification
string battery_name

# Voltage and current
float64 voltage           # Battery voltage in Volts
float64 current           # Battery current in Amperes (+ charging, - discharging)
float64 power             # Power in Watts

# Capacity information
float64 capacity          # Total capacity in Ah
float64 remaining_capacity # Remaining capacity in Ah
float64 percentage        # State of charge percentage (0-100)

# Temperature
float64 temperature       # Battery temperature in Celsius

# Status flags
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

uint8 power_supply_status

# Health information
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

uint8 power_supply_health

# Protection circuit status
bool protection_circuit_active
bool overvoltage_protection
bool undervoltage_protection
bool overcurrent_protection
bool short_circuit_protection

# Estimated time
builtin_interfaces/Duration time_to_empty
builtin_interfaces/Duration time_to_full
