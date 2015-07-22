
uint64 timestamp	# Microseconds since system boot
bool armed		# Set to true if system is armed
bool prearmed		# Set to true if the actuator safety is disabled but motors are not armed
bool ready_to_arm	# Set to true if system is ready to be armed
bool lockdown		# Set to true if actuators are forced to being disabled (due to emergency or HIL)
bool force_failsafe	# Set to true if the actuators are forced to the failsafe position
bool in_esc_calibration_mode # IO/FMU should ignore messages from the actuator controls topics
