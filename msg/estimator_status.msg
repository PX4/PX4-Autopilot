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
# 5 - true if 3-axis magnetometer measurement are being fused
# 6 - true if synthetic magnetic declination measurements are being fused
# 7 - true when the vehicle is airborne
# 8 - true when wind velocity is being estimated
# 9 - true when baro height is being fused as a primary height reference
# 10 - true when range finder height is being fused as a primary height reference
# 11 - true when range finder height is being fused as a primary height reference
uint16 filter_fault_flags	# Bitmask to indicate EKF internal faults
# 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
# 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
# 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
# 3 - true if the fusion of the magnetic heading has encountered a numerical error
# 4 - true if the fusion of the magnetic declination has encountered a numerical error
# 5 - true if fusion of the airspeed has encountered a numerical error
# 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
# 7 - true if fusion of the optical flow X axis has encountered a numerical error
# 8 - true if fusion of the optical flow Y axis has encountered a numerical error
# 9 - true if fusion of the North velocity has encountered a numerical error
# 10 - true if fusion of the East velocity has encountered a numerical error
# 11 - true if fusion of the Down velocity has encountered a numerical error
# 12 - true if fusion of the North position has encountered a numerical error
# 13 - true if fusion of the East position has encountered a numerical error
# 14 - true if fusion of the Down position has encountered a numerical error
float32 pos_horiz_accuracy
float32 pos_vert_accuracy