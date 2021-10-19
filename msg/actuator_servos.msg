# Servo control message
uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

uint8 NUM_CONTROLS = 8
float32[8] control # range: [-1, 1], where 1 means maximum positive position,
                   # -1 maximum negative,
                   # and NaN maps to disarmed

