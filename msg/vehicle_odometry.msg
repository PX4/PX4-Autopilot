# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

# Position and linear velocity frame of reference constants
uint8 FRAME_UNKNOWN     = 0
uint8 LOCAL_FRAME_NED   = 1 # NED earth-fixed frame
uint8 LOCAL_FRAME_FRD   = 2 # FRD earth-fixed frame, arbitrary heading reference
uint8 BODY_FRAME_FRD    = 3 # FRD body-fixed frame

# Position frame of reference
uint8 local_frame

float32[3] position # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown

# Orientation quaternion. First value NaN if invalid/unknown
float32[4] q			# Quaternion rotation from FRD body frame to reference frame

# Row-major representation of 3x3 position cross-covariance matrix URT.
# Order: x, y, z
uint8 POSITION_COVARIANCE_X_VAR  = 0
uint8 POSITION_COVARIANCE_XY_COV = 1
uint8 POSITION_COVARIANCE_XZ_COV = 2
uint8 POSITION_COVARIANCE_Y_VAR  = 3
uint8 POSITION_COVARIANCE_YZ_COV = 4
uint8 POSITION_COVARIANCE_Z_VAR  = 5
float32[6] position_covariance

# Row-major representation of 3x3 orientation cross-covariance matrix URT.
# Order: roll, pitch, yaw
uint8 ORIENTATION_COVARIANCE_R_VAR  = 0
uint8 ORIENTATION_COVARIANCE_RP_COV = 1
uint8 ORIENTATION_COVARIANCE_RY_COV = 2
uint8 ORIENTATION_COVARIANCE_P_VAR  = 3
uint8 ORIENTATION_COVARIANCE_PY_COV = 4
uint8 ORIENTATION_COVARIANCE_Y_VAR  = 5
float32[6] orientation_covariance

# Reference frame of the velocity data
uint8 velocity_frame

float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown

float32[3] angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

# Row-major representation of 3x3 velocity cross-covariance matrix URT.
# Order: vx, vy, vz
uint8 VELOCITY_COVARIANCE_VX_VAR   = 0
uint8 VELOCITY_COVARIANCE_VXVY_COV = 1
uint8 VELOCITY_COVARIANCE_VXVZ_COV = 2
uint8 VELOCITY_COVARIANCE_VY_VAR   = 3
uint8 VELOCITY_COVARIANCE_VYVZ_COV = 4
uint8 VELOCITY_COVARIANCE_VZ_VAR   = 5
float32[6] velocity_covariance

uint8 reset_counter
int8 quality

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry estimator_visual_odometry_aligned
