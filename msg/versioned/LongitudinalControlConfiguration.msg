# Fixed Wing Longitudinal Control Configuration message
# Used by the fw_lateral_longitudinal_control module and TECS to constrain FixedWingLongitudinalSetpoint messages
# and configure the resultant setpoints.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 pitch_min   			# [rad][@range -pi, pi] defaults to FW_P_LIM_MIN if NAN.
float32 pitch_max   			# [rad][@range -pi, pi] defaults to FW_P_LIM_MAX if NAN.
float32 throttle_min 			# [norm] [@range 0,1] deaults to FW_THR_MIN if NAN.
float32 throttle_max 			# [norm] [@range 0,1] defaults to FW_THR_MAX if NAN.
float32 climb_rate_target 		# [m/s] target climbrate to change altitude. Defaults to FW_T_CLIMB_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.
float32 sink_rate_target 		# [m/s] target sinkrate to change altitude. Defaults to FW_T_SINK_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.
float32 speed_weight 			# [@range 0,2], 0=pitch controls altitude only, 2=pitch controls airspeed only
bool enforce_low_height_condition 	# [boolean] if true, the altitude controller is configured with an alternative timeconstant for tighter altitude tracking
bool disable_underspeed_protection 	# [boolean] if true, underspeed handling is disabled in the altitude controller
