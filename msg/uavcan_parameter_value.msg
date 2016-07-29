# UAVCAN-MAVLink parameter bridge response type
uint8 node_id			# UAVCAN node ID mapped from MAVLink component ID
char[17] param_id		# MAVLink/UAVCAN parameter name
int16 param_index		# parameter index, if known
uint16 param_count		# number of parameters exposed by the node
uint8 param_type		# MAVLink parameter type
int64 int_value			# current value if param_type is int-like
float32 real_value		# current value if param_type is float-like
