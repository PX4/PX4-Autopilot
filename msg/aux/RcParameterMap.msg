uint64 timestamp		# time since system start (microseconds)
uint8 RC_PARAM_MAP_NCHAN = 3 # This limit is also hardcoded in the enum RC_CHANNELS_FUNCTION in rc_channels.h
uint8 PARAM_ID_LEN = 16 # corresponds to MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN

bool[3] valid		#true for RC-Param channels which are mapped to a param
int32[3] param_index	# corresponding param index, this field is ignored if set to -1, in this case param_id will be used
char[51] param_id	# MAP_NCHAN * (ID_LEN + 1) chars, corresponding param id, null terminated
float32[3] scale		# scale to map the RC input [-1, 1] to a parameter value
float32[3] value0		# initial value around which the parameter value is changed
float32[3] value_min	# minimal parameter value
float32[3] value_max	# minimal parameter value
