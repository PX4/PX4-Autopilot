
uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz		# angular acceleration about X, Y, Z body axis in rad/s^2,
			# computed by differentiating vehicle_angular_velocity and applying a low pass filter
			# can be used to verify that the vehicle correctly tracks angular acceleration setpoints
