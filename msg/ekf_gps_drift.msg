uint64 timestamp		# time since system start (microseconds)
float32 hpos_drift_rate		# Horizontal position rate magnitude checked using EKF2_REQ_HDRIFT (m/s)
float32 vpos_drift_rate		# Vertical position rate magnitude checked using EKF2_REQ_VDRIFT (m/s)
float32 hspd			# Filtered horizontal velocity magnitude checked using EKF2_REQ_HDRIFT (m/s)
bool blocked			# true when drift calculation is blocked due to IMU movement check controlled by EKF2_MOVE_TEST
