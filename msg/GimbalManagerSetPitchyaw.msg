# Pitch/yaw setpoint sent by PX4 (acting as a gimbal manager client) to an
# external gimbal manager. Streamed out as GIMBAL_MANAGER_SET_PITCHYAW.

uint64 timestamp		# time since system start (microseconds)

uint8 target_system		# system id of the external gimbal manager
uint8 target_component	# component id of the external gimbal manager
uint8 gimbal_device_id	# gimbal device to address (0 for all)

uint32 flags			# GIMBAL_MANAGER_FLAGS bitmask

uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
uint32 GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64

float32 pitch			# [rad] pitch angle (NaN to be ignored)
float32 yaw				# [rad] yaw angle (NaN to be ignored)
float32 pitch_rate		# [rad/s] pitch angular rate (NaN to be ignored)
float32 yaw_rate		# [rad/s] yaw angular rate (NaN to be ignored)
