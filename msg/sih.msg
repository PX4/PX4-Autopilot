# simulator in Hardware - Romain Chiappinelli - Jan 8, 2019
uint64 timestamp				# time since system start (microseconds)

uint32      dt_us 			# simulator sampling time [us]
float32[3]  euler_rpy 		# euler angles (roll-pitch-yaw) [deg]
float32[3] 	omega_b			# body rates in body frame [rad/s]
float32[3]  p_i_local 		# local inertial position [m]
float32[3]  v_i 			# inertial velocity [m]
float32[4]  u 				# motor signals [0;1]
uint32     te_us 			# execution time [us]
uint32     td_us 			# delay time [us]
