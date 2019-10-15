uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp_sample	# time since system start (microseconds)

float32 dt		# delta time between samples (microseconds)
float32 scale

uint8 samples		# number of valid samples

int16[4] x		# acceleration in the NED X board axis in m/s/s
int16[4] y		# acceleration in the NED Y board axis in m/s/s
int16[4] z		# acceleration in the NED Z board axis in m/s/s
