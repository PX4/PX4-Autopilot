#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32 accel_device_id_primary       # current primary accel device id for reference

uint32[3] accel_device_ids
float32[3] accel_inconsistency_m_s_s # magnitude of acceleration difference between IMU instance and primary in (m/s/s).
bool[3] accel_healthy


uint32 gyro_device_id_primary       # current primary gyro device id for reference

uint32[3] gyro_device_ids
float32[3] gyro_inconsistency_rad_s # magnitude of angular rate difference between IMU instance and primary in (rad/s).
bool[3] gyro_healthy
