#Bridge private set-attitude command for an external gimbal manager
uint64 timestamp						# time since system start (microseconds)

uint8 origin_sysid
uint8 origin_compid

uint8 target_system
uint8 target_component

uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_RETRACT = 1
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_NEUTRAL = 2
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME = 32
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME = 128
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE = 256
uint32 EXTERNAL_GIMBAL_MANAGER_FLAGS_RC_MIXED = 512

uint32 flags
uint8 gimbal_device_id

float32[4] q

float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

uint8 ORB_QUEUE_LENGTH = 2
