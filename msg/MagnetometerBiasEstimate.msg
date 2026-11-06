uint64 timestamp                # time since system start (microseconds)

float32[4] bias_x		# estimated X-bias of all the sensors
float32[4] bias_y		# estimated Y-bias of all the sensors
float32[4] bias_z		# estimated Z-bias of all the sensors

bool[4] valid			# true if the estimate has converged on the observed axes, unobserved axes are 0
bool[4] stable			# true if valid for at least 5 s
