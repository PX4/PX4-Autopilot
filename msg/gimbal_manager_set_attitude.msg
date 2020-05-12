uint64 timestamp						# time since system start (microseconds)

uint8 target_system
uint8 target_component

uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
uint32 GIMBAL_MANAGER_FLAGS_ANGULAR_VELOCITY_RELATIVE_TO_FOCAL_LENGTH = 1048576
uint32 GIMBAL_MANAGER_FLAGS_NUDGE = 2097152
uint32 GIMBAL_MANAGER_FLAGS_OVERRIDE = 4194304
uint32 GIMBAL_MANAGER_FLAGS_NONE = 8388608

uint32 flags
uint8 gimbal_device_id

float32[4] q

float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z
