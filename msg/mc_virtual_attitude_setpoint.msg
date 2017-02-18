###############################################################################################
# The vehicle_attitude_setpoint.msg needs to be in sync with the virtual setpoint messages
#
# Please keep the following messages identical;
#	vehicle_attitude_setpoint.msg
#	mc_virtual_attitude_setpoint.msg
#	fw_virtual_attitude_setpoint.msg
#
###############################################################################################


float32 roll_body				# body angle in NED frame
float32 pitch_body				# body angle in NED frame
float32 yaw_body				# body angle in NED frame

float32 yaw_sp_move_rate		# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d					# Desired quaternion for quaternion control
bool q_d_valid					# Set to true if quaternion vector is valid

float32 thrust					# Thrust in Newton the power system should generate

bool roll_reset_integral			# Reset roll integral part (navigation logic change)
bool pitch_reset_integral			# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral				# Reset yaw integral part (navigation logic change)

bool fw_control_yaw					# control heading with rudder (used for auto takeoff on runway)
bool disable_mc_yaw_control			# control yaw for mc (used for vtol weather-vane mode)

bool apply_flaps

float32 landing_gear

# WAS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
# TOPICS mc_virtual_attitude_setpoint
