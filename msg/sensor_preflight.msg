#
# Pre-flight sensor state. Inconsistency metrics will be zero if the vehicle only has one sensor.
# The topic will not be updated when the vehicle is armed
#
uint64 timestamp # time since system start (microseconds)

float32 accel_inconsistency_m_s_s # magnitude of maximum acceleration difference between IMU instances in (m/s/s).
float32 gyro_inconsistency_rad_s # magnitude of maximum angular rate difference between IMU instances in (rad/s).
float32 mag_inconsistency_angle # maximum angle between magnetometer instance field vectors in radians.

float32 accel_magnitude # magnitude of the acceleration vector

bool gyro_prime_healthy # flag indicating if the primary gyro is healthy
bool accel_prime_healthy # flag indicating if the primary accel is healthy
bool mag_prime_healthy # flag indicating if the primary mag is healthy
bool baro_prime_healthy # flag indicating if the primary baro is healthy

uint64 calibrated # bitmask indicating if the sensor is calibrated. The bitmask is defined in the subsystem_info message
