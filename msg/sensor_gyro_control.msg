
# WARNING: Not recommend for custom development.
#  This topic is part of an incremental refactoring of the sensor pipeline and will likely disappear post PX4 release v1.10

uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 timestamp	# time since system start (microseconds)

uint64 timestamp_sample	# time the raw data was sampled (microseconds)

float32[3] xyz		# filtered angular velocity in the board axis in rad/s
