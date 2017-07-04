float32[24] states		# Internal filter states
float32 n_states		# Number of states effectively used
float32[3] vibe			# IMU vibration metrics in the following array locations
# 0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
# 1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
# 2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
uint8 nan_flags			# Bitmask to indicate NaN states
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)
float32[24] covariances	# Diagonal Elements of Covariance Matrix
uint16 gps_check_fail_flags     # Bitmask to indicate status of GPS checks - see definition below
# bits are true when corresponding test has failed
uint16 GPS_CHECK_FAIL_MIN_SAT_COUNT = 0			# 0 : minimum required sat count fail
uint16 GPS_CHECK_FAIL_MIN_GDOP = 1			# 1 : minimum required GDoP fail
uint16 GPS_CHECK_FAIL_MAX_HORZ_ERR = 2			# 2 : maximum allowed horizontal position error fail
uint16 GPS_CHECK_FAIL_MAX_VERT_ERR = 3			# 3 : maximum allowed vertical position error fail
uint16 GPS_CHECK_FAIL_MAX_SPD_ERR = 4			# 4 : maximum allowed speed error fail
uint16 GPS_CHECK_FAIL_MAX_HORZ_DRIFT = 5		# 5 : maximum allowed horizontal position drift fail
uint16 GPS_CHECK_FAIL_MAX_VERT_DRIFT = 6		# 6 : maximum allowed vertical position drift fail
uint16 GPS_CHECK_FAIL_MAX_HORZ_SPD_ERR = 7		# 7 : maximum allowed horizontal velocity discrepancy fail
uint16 GPS_CHECK_FAIL_MAX_VERT_SPD_ERR = 8		# 8 : maximum allowed vertical velocity discrepancy fail

uint32 control_mode_flags	# Bitmask to indicate EKF logic state
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
# 11 - true when GPS height is being fused as a primary height reference
# 12 - true when local position data from external vision is being fused
# 13 - true when yaw data from external vision measurements is being fused
# 14 - true when height data from external vision measurements is being fused
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
float32 pos_horiz_accuracy # 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)
float32 pos_vert_accuracy # 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)
uint16 innovation_check_flags # Bitmask to indicate pass/fail status of innovation consistency checks
# 0 - true if velocity observations have been rejected
# 1 - true if horizontal position observations have been rejected
# 2 - true if true if vertical position observations have been rejected
# 3 - true if the X magnetometer observation has been rejected
# 4 - true if the Y magnetometer observation has been rejected
# 5 - true if the Z magnetometer observation has been rejected
# 6 - true if the yaw observation has been rejected
# 7 - true if the airspeed observation has been rejected
# 8 - true if the synthetic sideslip observation has been rejected
# 9 - true if the height above ground observation has been rejected
# 10 - true if the X optical flow observation has been rejected
# 11 - true if the Y optical flow observation has been rejected
float32 mag_test_ratio # ratio of the largest magnetometer innovation component to the innovation test limit
float32 vel_test_ratio # ratio of the largest velocity innovation component to the innovation test limit
float32 pos_test_ratio # ratio of the largest horizontal position innovation component to the innovation test limit
float32 hgt_test_ratio # ratio of the vertical position innovation to the innovation test limit
float32 tas_test_ratio # ratio of the true airspeed innovation to the innovation test limit
float32 hagl_test_ratio # ratio of the height above ground innovation to the innovation test limit
uint16 solution_status_flags # Bitmask indicating which filter kinematic state outputs are valid for flight control use.
# 0 - True if the attitude estimate is good
# 1 - True if the horizontal velocity estimate is good
# 2 - True if the vertical velocity estimate is good
# 3 - True if the horizontal position (relative) estimate is good
# 4 - True if the horizontal position (absolute) estimate is good
# 5 - True if the vertical position (absolute) estimate is good
# 6 - True if the vertical position (above ground) estimate is good
# 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
# 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
# 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
# 10 - True if the EKF has detected a GPS glitch
float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time
