uint64 timestamp					# time since system start (microseconds)

# From the StatusExtended.uavcan message
uint8 input_percent					# Input command to ESC, in percent, which is commanded using the setpoint messages. Range 0% to 100%.
uint8 output_percent					# Output command from ESC to motor, in percent. Range 0% to 100%.
int16 motor_temperature_deg_c  				# Temperature of connected motor, in Celsius. Range is -256 to +255 C.
uint16 motor_angle             				# Measured angle of connected angle sensor, in degrees. Range is 0 to 360.
uint32 status_flags           				# Manufacturer-specific status flags currently active.
uint8 esc_index               				# Index of currently reporting ESC.
