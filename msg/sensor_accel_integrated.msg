uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_sample	# time since system start (microseconds)

uint16 dt	# integration time (microseconds)

float32[3] delta_velocity	# delta velocity in the NED frame in m/s over the integration time frame (dt)

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 clipping		# accelerometer was clipping during integration period
