# Gimbal manager pitch/yaw setpoint 
#
# This is sent by PX4 (acting as a gimbal manager client) to an
# external gimbal manager). Streamed out as GIMBAL_MANAGER_SET_PITCHYAW.

uint64 timestamp # [us] Time since system start

uint8 target_system # [-] System id of the external gimbal manager
uint8 target_component	# [-] Component id of the external gimbal manager
uint8 gimbal_device_id # [-] Gimbal device to address (0 for all)

uint32 flags # [@enum GIMBAL_MANAGER_FLAGS] GIMBAL_MANAGER_FLAGS bitmask
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
uint32 GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64

float32 pitch # [rad] [@invalid NaN to ignore] pitch angle
float32 yaw # [rad] [@invalid NaN to ignore] yaw angle
float32 pitch_rate # [rad/s] [@invalid NaN to ignore] Pitch angular rate
float32 yaw_rate # [rad/s] [@invalid NaN to ignore] Yaw angular rate
