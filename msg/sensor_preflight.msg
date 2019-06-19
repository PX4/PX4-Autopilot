#
# Pre-flight sensor check metrics. These will be zero if the vehicle only has one sensor.
# The topic will not be updated when the vehicle is armed
#
uint64 timestamp		   # time since system start (microseconds)
float32 accel_inconsistency_m_s_s  # magnitude of maximum acceleration difference between IMU instances in (m/s/s).
float32 gyro_inconsistency_rad_s   # magnitude of maximum angular rate difference between IMU instances in (rad/s).
float32 mag_inconsistency_h        # magnitude of maximum horizontal angular difference between Mag instances in (rad).
float32 mag_inconsistency_v        # magnitude of maximum vertical (inclination) angular difference between Mag instances in (rad).
