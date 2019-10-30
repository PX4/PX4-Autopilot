uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_sample	# time since system start (microseconds)

uint16 dt	# integration time (microseconds)

float32[3] delta_angle	# delta angle in the NED frame in rad/s over the integration time frame (integral_dt)

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4

uint8 clipping		# gyroscope was clipping during integration period
