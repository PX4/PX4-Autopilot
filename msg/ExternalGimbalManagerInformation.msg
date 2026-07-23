# Information about a gimbal manager running on another component (not us),
# decoded from an incoming GIMBAL_MANAGER_INFORMATION message. Used to discover
# external gimbal managers and their capabilities.

uint64 timestamp						# time since system start (microseconds)

uint8 manager_sysid						# system id of the external gimbal manager
uint8 manager_compid					# component id of the external gimbal manager

uint32 cap_flags						# GIMBAL_MANAGER_CAP_FLAGS bitmask

uint8 gimbal_device_id					# gimbal device managed by this manager

float32 roll_min # [rad]
float32 roll_max # [rad]

float32 pitch_min # [rad]
float32 pitch_max # [rad]

float32 yaw_min # [rad]
float32 yaw_max # [rad]
