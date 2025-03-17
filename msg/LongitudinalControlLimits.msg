uint64 timestamp

float32 pitch_min   			# [rad]
float32 pitch_max   			# [rad]
float32 throttle_min 			# [0,1]
float32 throttle_max 			# [0,1]
float32 climb_rate_target 		# target climbrate used to change altitude, [m/s]
float32 sink_rate_target 		# target sinkrate used to change altitude, [m/s]
float32 equivalent_airspeed_min 	# [m/s]
float32 equivalent_airspeed_max 	# [m/s]
float32 speed_weight 			# [0,2], 0=pitch controls altitude only, 2=pitch controls airspeed only
bool enforce_low_height_condition 	# if true, total energy controller will use lower altitude control time constant
bool disable_underspeed_protection 	#
