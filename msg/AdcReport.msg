uint64 timestamp		# time since system start (microseconds)
uint32 device_id		# unique device ID for the sensor that does not change between power cycles
int16[12] channel_id		# ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
int32[12] raw_data		# ADC channel raw value, accept negative value, valid if channel ID is positive
float32[12] raw_data_ext	# External ADC raw data (used only because we use float data type instead of int16)
uint32 resolution		# ADC channel resolution
float32 v_ref			# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)

# TOPICS adc_report
# TOPICS external_adc
