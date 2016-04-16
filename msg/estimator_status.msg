uint64 timestamp		# Timestamp in microseconds since boot
float32[32] states		# Internal filter states
float32 n_states		# Number of states effectively used
float32[3] vibe			# Vibration levels in X, Y and Z
uint8 nan_flags			# Bitmask to indicate NaN states
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)
float32[28] covariances	# Diagonal Elements of Covariance Matrix
uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below
# bits are true when corresponding test has failed
# 0 : minimum required sat count fail
# 1 : minimum required GDoP fail
# 2 : maximum allowed horizontal position error fail
# 3 : maximum allowed vertical position error fail
# 4 : maximum allowed speed error fail
# 5 : maximum allowed horizontal position drift fail
# 6 : maximum allowed vertical position drift fail
# 7 : maximum allowed horizontal speed fail
# 8 : maximum allowed vertical velocity discrepancy fail
uint16 control_mode_flags	# Bitmask to indicate EKF logic state
# 0 - true if the filter tilt alignment is complete
# 1 - true if the filter yaw alignment is complete
# 2 - true if GPS measurements are being fused
# 3 - true if optical flow measurements are being fused
# 4 - true if a simple magnetic yaw heading is being fused
# 5 - true if the horizontal projection of magnetometer data is being fused
# 6 - true if 3-axis magnetometer measurement are being fused
# 7 - true if synthetic magnetic declination measurements are being fused
# 8 - true when the vehicle is airborne
# 9 - true when the vehicle motors are armed
# 10 - true when wind velocity is being estimated
# 11 - true when baro height is being fused as a primary height reference
# 12 - true when range finder height is being fused as a primary height reference
# 15 - true when range finder height is being fused as a primary height reference
