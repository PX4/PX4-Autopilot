uint64 timestamp					# time since system start (microseconds)

uint8 CONNECTED_ESC_MAX = 8					# The number of ESCs supported. To be consistent with ESC Status, limit it to 8.

# From the StatusExtended.uavcan message
uint8 esc_index               				# Index of currently reporting ESC.
uint8 input_percent					# Input command to ESC, in percent, which is commanded using the setpoint messages. Range 0% to 100%.
uint8 output_percent					# Output command from ESC to motor, in percent. Range 0% to 100%.
int16 motor_temperature_deg_c  				# Temperature of connected motor, in Celsius. Range is -256 to +255 C.
uint16 motor_angle             				# Measured angle of connected angle sensor, in degrees. Range is 0 to 360.
uint32 status_flags           				# Manufacturer-specific status flags currently active.
