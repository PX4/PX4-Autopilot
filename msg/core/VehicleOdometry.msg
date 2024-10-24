# Vehicle odometry data. Fits ROS REP 147 for aerial vehicles

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint8 POSE_FRAME_UNKNOWN = 0
uint8 POSE_FRAME_NED     = 1 # NED earth-fixed frame
uint8 POSE_FRAME_FRD     = 2 # FRD world-fixed frame, arbitrary heading reference
uint8 pose_frame            # Position and orientation frame of reference

float32[3] position         # Position in meters. Frame of reference defined by local_frame. NaN if invalid/unknown
float32[4] q                # Quaternion rotation from FRD body frame to reference frame. First value NaN if invalid/unknown

uint8 VELOCITY_FRAME_UNKNOWN  = 0
uint8 VELOCITY_FRAME_NED      = 1 # NED earth-fixed frame
uint8 VELOCITY_FRAME_FRD      = 2 # FRD world-fixed frame, arbitrary heading reference
uint8 VELOCITY_FRAME_BODY_FRD = 3 # FRD body-fixed frame
uint8 velocity_frame        # Reference frame of the velocity data

float32[3] velocity         # Velocity in meters/sec. Frame of reference defined by velocity_frame variable. NaN if invalid/unknown

float32[3] angular_velocity # Angular velocity in body-fixed frame (rad/s). NaN if invalid/unknown

float32[3] position_variance
float32[3] orientation_variance
float32[3] velocity_variance

uint8 reset_counter
int8 quality

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry
