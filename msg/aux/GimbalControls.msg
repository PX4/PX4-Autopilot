uint64 timestamp			# time since system start (microseconds)
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2

uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[3] control
