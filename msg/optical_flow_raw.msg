# Optical flow in XYZ body frame in SI units.
# @see http://en.wikipedia.org/wiki/International_System_of_Units

# This message is raw in the sense of not yet rotated using SENS_FLOW_ROT

uint8 sensor_id			# id of the sensor emitting the flow value
float32 pixel_flow_x_integral	# accumulated optical flow in radians
float32 pixel_flow_y_integral	# accumulated optical flow in radians
float32 gyro_x_rate_integral	# accumulated gyro value in radians (same direction as pixel_flow_x_integral)
float32 gyro_y_rate_integral	# accumulated gyro value in radians (same direction as pixel_flow_y_integral)
float32 gyro_z_rate_integral	# accumulated gyro value in radians (has to form right handed coordinate system)
float32 ground_distance_m	# Altitude / distance to ground in meters
uint32 integration_timespan	# accumulation timespan in microseconds
uint32 time_since_last_sonar_update	# time since last sonar update in microseconds
uint16 frame_count_since_last_readout	# number of accumulated frames in timespan
int16 gyro_temperature	# Temperature * 100 in centi-degrees Celsius
uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality
