## Used to command the parachute output function, which is mapped to a specific output in the control allocation module.
## The release command is latching: once released, the parachute output stays released until reboot.

uint64 timestamp

int8 command		# Commanded parachute action, mirrors the MAVLink PARACHUTE_ACTION enum
int8 COMMAND_DISABLE = 0	# Inhibit the parachute subsystem: a release must not fire it
int8 COMMAND_ENABLE = 1		# Arm the parachute subsystem so that a release can fire it
int8 COMMAND_RELEASE = 2	# Fire the parachute. Latching: stays released until reboot
