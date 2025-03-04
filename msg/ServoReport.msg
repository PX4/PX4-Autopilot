uint64 timestamp					# time since system start (microseconds)
uint8 servo_actuator_id					# actuator id of current servo (in most cases 1-8)
uint8 servo_node_id					# Node ID of device on CAN bus
float32 servo_position					# meter or radian
float32 servo_force					# Newton or Newton metre
float32 servo_speed					# meter per second or radian per second
uint8 servo_power_rating_pct				# 0 - unloaded, 100 - full load

uint8 servo_function					# servo output function


float32 servo_temperature				# in kelvin
uint8 servo_temperature_error_flags

uint8 ERROR_FLAG_OVERHEATING = 1
uint8 ERROR_FLAG_OVERCOOLING = 2

float32 servo_voltage					# Volts
float32 servo_current					# Amps
uint8 servo_power_error_flags

uint8 ERROR_FLAG_OVERVOLTAGE  = 1
uint8 ERROR_FLAG_UNDERVOLTAGE = 2
uint8 ERROR_FLAG_OVERCURRENT  = 4
uint8 ERROR_FLAG_UNDERCURRENT = 8
