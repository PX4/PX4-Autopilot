uint8 NUM_ACTUATOR_CONTROLS = 8
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 4
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 INDEX_FLAPS = 4
uint8 GROUP_INDEX_ATTITUDE = 0
uint64 timestamp
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[8] control
