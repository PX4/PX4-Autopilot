uint8 NUM_ACTUATOR_OUTPUTS		= 16
uint8 NUM_ACTUATOR_OUTPUT_GROUPS	= 4	# for sanity checking
uint32 noutputs				# valid outputs
float32[16] output			# output data, in natural output units

uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
