# Vehicle full state information data for ROS2-based control loops
uint32 MESSAGE_VERSION = 0

uint64 timestamp              # hrt_absolute_time()
uint64 timestamp_sample       # same timing as odometry: imu_sample.time_us

# local frame is NED (same as vehicle_odometry/vehicle_local_position)
float32[3] position           # (x,y,z) in meters, NED
float32[3] velocity           # (vx,vy,vz) in m/s, NED
float32[3] acceleration       # (ax,ay,az) in m/s^2, NED (linear accel)
float32[4] q                  # quaternion (w,x,y,z): FRD body -> NED
float32[3] angular_velocity   # (p,q,r) in rad/s, body frame (FRD)
