# Motor control message
uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

uint16 reversible_flags     # bitset which motors are configured to be reversible

uint8 NUM_CONTROLS = 8
float32[8] control # range: [-1, 1], where 1 means maximum positive thrust,
                   # -1 maximum negative (if not supported by the output, <0 maps to NaN),
                   # and NaN maps to disarmed (stop the motors)
