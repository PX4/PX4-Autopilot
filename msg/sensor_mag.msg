uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 x		# magnetic field in the NED X board axis in Gauss
float32 y		# magnetic field in the NED Y board axis in Gauss
float32 z		# magnetic field in the NED Z board axis in Gauss

float32 temperature	# temperature in degrees celsius

float32 scaling		# scaling from raw to Gauss
int16 x_raw
int16 y_raw
int16 z_raw

bool  is_external	# if true the mag is external (i.e. not built into the board)
