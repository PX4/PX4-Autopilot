uint64 timestamp		# time since system start (microseconds)
uint32 device_id		# unique device ID for the sensor that does not change between power cycles
int16[12] channel_id		# ADC channel IDs, negative for non-existent, should be kept same as array index
int32[12] raw_data		# ADC channel raw value, accept negative value, valid if channel ID is positive
uint8[12] resolution		# ADC channel resolution, valid if channel ID is positive
float32[12] scale		# ADC channel voltage scale, use to calculate LSB voltage(lsb=scale/resolution), valid if channel ID is positive