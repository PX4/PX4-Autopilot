# parameter response type
uint64 timestamp		# time since system start (microseconds)

uint8 node_id			# requester node ID

char[17] param_name		# parameter name

int16 param_index		# parameter index, if known

uint16 param_count		# number of parameters exposed by the node

uint8 type                      # parameter type

int32 int32_value               # current value if param_type is int-like
float32 float32_value           # current value if param_type is float-like


uint64 timestamp_requested

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS parameter_value uavcan_parameter_value
