uint64 timestamp						# time since system start (microseconds)

uint8[32] vendor_name
uint8[32] model_name
uint32 firmware_version

uint16 capability_flags

uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024

float32 pitch_max # [rad]
float32 pitch_min # [rad]

float32 yaw_max # [rad]
float32 yaw_min # [rad]

uint8 gimbal_device_compid
