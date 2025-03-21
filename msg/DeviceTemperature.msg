uint64 timestamp			# time since system start (microseconds)
uint8 actuator_id			# actuator id of current servo (in most cases 1-8)
uint8 node_id				# Node ID of device on CAN bus
float32 temperature			# in kelvin
uint8 error_flags

uint8 ERROR_FLAG_OVERHEATING = 1
uint8 ERROR_FLAG_OVERCOOLING = 2
