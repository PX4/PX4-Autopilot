int16 RANGE_FINDER_TYPE_LASER = 0

# range finder report structure.  Reads from the device must be in multiples of this
# structure.
uint64 timestamp
uint64 error_count
uint16 type				# type, following RANGE_FINDER_TYPE enum 
float32 distance			# in meters
float32 minimum_distance			# minimum distance the sensor can measure
float32 maximum_distance			# maximum distance the sensor can measure
uint8 valid				# 1 == within sensor range, 0 = outside sensor range
float32[12] distance_vector # in meters length should match MB12XX_MAX_RANGEFINDERS
uint8 just_updated # number of the most recent measurement sensor
