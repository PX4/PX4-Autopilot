# TOPICS raptor_status
uint64 timestamp
uint64 timestamp_sample
bool subscription_update_angular_velocity
bool subscription_update_local_position
bool subscription_update_attitude
bool subscription_update_trajectory_setpoint
bool subscription_update_vehicle_status

uint8 exit_reason
uint8 EXIT_REASON_NONE                               = 0
uint8 EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE         = 1
uint8 EXIT_REASON_NOT_ALL_OBSERVATIONS_SET           = 2
uint8 EXIT_REASON_ANGULAR_VELOCITY_STALE             = 3
uint8 EXIT_REASON_LOCAL_POSITION_STALE               = 4
uint8 EXIT_REASON_VISUAL_ODOMETRY_STALE              = 5
uint8 EXIT_REASON_VISUAL_ODOMETRY_WRONG_FRAME        = 6
uint8 EXIT_REASON_VISUAL_ODOMETRY_NAN                = 7
uint8 EXIT_REASON_ATTITUDE_STALE                     = 8
uint8 EXIT_REASON_EXECUTOR_STATUS_SOURCE_NOT_CONTROL = 9

uint32 timestamp_last_vehicle_angular_velocity
uint32 timestamp_last_vehicle_local_position
uint32 timestamp_last_vehicle_attitude
uint32 timestamp_last_trajectory_setpoint

bool vehicle_angular_velocity_stale
bool vehicle_local_position_stale
bool vehicle_attitude_stale
bool trajectory_setpoint_stale

bool active
uint8 substep
float32 control_interval
