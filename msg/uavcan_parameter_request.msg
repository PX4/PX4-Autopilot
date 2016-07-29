# UAVCAN-MAVLink parameter bridge request type
uint8 message_type		# MAVLink message type: PARAM_REQUEST_READ, PARAM_REQUEST_LIST, PARAM_SET
uint8 node_id			# UAVCAN node ID mapped from MAVLink component ID
char[17] param_id		# MAVLink/UAVCAN parameter name
int16 param_index		# -1 if the param_id field should be used as identifier
uint8 param_type		# MAVLink parameter type
int64 int_value			# current value if param_type is int-like
float32 real_value		# current value if param_type is float-like
