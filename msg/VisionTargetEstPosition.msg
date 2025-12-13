# Target estimator Kalman Filter state

uint64 timestamp		# time since system start (microseconds)

bool rel_pos_valid		# Flag showing whether relative position is valid
bool rel_vel_valid		# Flag showing whether relative velocity is valid

float32[3] rel_pos		# [x,y,z] NED position of target, relative to vehicle [m]
float32[3] vel_uav		# [x,y,z] NED velocity of vehicle [m/s]
float32[3] vel_target		# [x,y,z] NED velocity of target [m/s]
float32[3] bias			# [x,y,z] NED GNSS bias between vehicle and target receivers [m]
float32[3] acc_target		# [x,y,z] NED acceleration of target [m/s^2]

float32[3] cov_rel_pos		# [x,y,z] Position variance [m^2]
float32[3] cov_vel_uav		# [x,y,z] Vehicle velocity variance [(m/s)^2]
float32[3] cov_bias		# [x,y,z] Bias variance [m^2]
float32[3] cov_vel_target	# [x,y,z] Target velocity variance [(m/s)^2]
float32[3] cov_acc_target	# [x,y,z] Target acceleration variance [(m/s^2)^2]
