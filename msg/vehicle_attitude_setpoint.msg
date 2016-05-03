###############################################################################################
# The vehicle_attitude_setpoint.msg needs to be in sync with the virtual setpoint messages
#
# Please keep the following messages identical;
#	vehicle_attitude_setpoint.msg
#	mc_virtual_attitude_setpoint.msg
#	fw_virtual_attitude_setpoint.msg
#
###############################################################################################

uint64 timestamp				# in microseconds since system start, is set whenever the writing thread stores new data

float32 yaw_sp_move_rate		# rad/s (commanded by user)

float32[4] q_d					# desired quaternion for quaternion control
bool q_d_valid					# set to true if quaternion vector is valid

float32 thrust					# normalized thrust value between [0,1]

bool roll_reset_integral			# reset roll integral part (navigation logic change)
bool pitch_reset_integral			# reset pitch integral part (navigation logic change)
bool yaw_reset_integral				# reset yaw integral part (navigation logic change)

bool fw_control_yaw					# control heading with rudder (used for auto takeoff on runway)
bool disable_mc_yaw_control			# control yaw for mc (used for vtol weather-vane mode)
bool apply_flaps
