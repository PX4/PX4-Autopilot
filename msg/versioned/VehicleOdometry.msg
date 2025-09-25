# Vehicle odometry data
#
# Fits ROS REP 147 for aerial vehicles

uint32 MESSAGE_VERSION = 0

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Timestamp sample

uint8 pose_frame              # [@enum POSE_FRAME] Position and orientation frame of reference
uint8 POSE_FRAME_UNKNOWN = 0
uint8 POSE_FRAME_NED     = 1  # NED earth-fixed frame. Origin is position of GC at startup.
uint8 POSE_FRAME_FRD     = 2  # FRD world-fixed frame. Arbitrary origin and heading reference. Z is down.

float32[3] position  # [m] [@frame @local_frame] [@invalid NaN If invalid/unknown] Position
float32[4] q         # [-] [@invalid NaN First value if invalid/unknown] Quaternion rotation from FRD body frame to reference frame. Follows the Hamiltonian convention (w, x, y, z, right-handed, passive rotations from body to world)

uint8 velocity_frame               # [@enum VELOCITY_FRAME] Reference frame of the velocity data
uint8 VELOCITY_FRAME_UNKNOWN  = 0  # Unknown frame
uint8 VELOCITY_FRAME_NED      = 1  # NED earth-fixed frame
uint8 VELOCITY_FRAME_FRD      = 2  # FRD world-fixed frame. Arbitrary heading reference. Z is down.
uint8 VELOCITY_FRAME_BODY_FRD = 3  # FRD body-fixed frame

float32[3] velocity          # [m/s] [@frame @velocity_frame] [@invalid NaN If invalid/unknown] Velocity.
float32[3] angular_velocity  # [rad/s] [@frame @VELOCITY_FRAME_BODY_FRD] [@invalid NaN If invalid/unknown] Angular velocity in body-fixed frame

float32[3] position_variance     # [m^2] Variance of position error
float32[3] orientation_variance  # [rad^2] Variance of orientation/attitude error (expressed in body frame)
float32[3] velocity_variance     # [m^2/s^2] Variance of velocity error

uint8 reset_counter  # [-] Reset counter. Counts reset events on attitude, velocity and position.
int8 quality         # [-] [@invalid 0] Quality. Unused.

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry
