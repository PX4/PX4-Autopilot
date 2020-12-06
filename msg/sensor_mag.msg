uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 x		# magnetic field in the FRD body frame X-axis in Gauss
float32 y		# magnetic field in the FRD body frame Y-axis in Gauss
float32 z		# magnetic field in the FRD body frame Z-axis in Gauss

float32 temperature	# temperature in degrees Celsius

float32 scaling		# scaling from raw to Gauss
int16 x_raw
int16 y_raw
int16 z_raw

bool  is_external	# if true the mag is external (i.e. not built into the board)
