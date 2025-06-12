# GimbalManagerInformation (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerInformation.msg)

```c
uint64 timestamp						# time since system start (microseconds)

uint32 cap_flags

uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024
uint32 GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072

uint8 gimbal_device_id

float32 roll_min # [rad]
float32 roll_max # [rad]

float32 pitch_min # [rad]
float32 pitch_max # [rad]

float32 yaw_min # [rad]
float32 yaw_max # [rad]

```
