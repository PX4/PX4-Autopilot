## Used to command the parachute output function, which is mapped to a specific output in the control allocation module.
## The release command is latching: once released, the parachute output stays released until reboot.

uint64 timestamp

int8 command		# Commanded parachute action
int8 COMMAND_DISABLE = 0
int8 COMMAND_ENABLE = 1
int8 COMMAND_RELEASE = 2
