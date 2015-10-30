uint8 NUM_ACTUATOR_OUTPUTS		= 16
uint8 NUM_ACTUATOR_OUTPUT_GROUPS	= 4	# for sanity checking
# timestamp (uint64,  output timestamp in us since system boot) will be generated automatically
uint32 noutputs				# valid outputs
float32[16] output			# output data, in natural output units
