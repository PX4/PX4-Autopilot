uint64 timestamp                # time since system start (microseconds)

# In flight learned sensor calibrations
uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles
float32[3] gyro_calibration            # gyroscope in-run calibration in body frame (rad/s)
float32[3] gyro_calibration_variance
bool gyro_calibration_valid

uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles
float32[3] accel_calibration           # accelerometer in-run calibration in body frame (m/s^2)
float32[3] accel_calibration_variance
bool accel_calibration_valid

uint32 mag_device_id            # unique device ID for the sensor that does not change between power cycles
float32[3] mag_calibration             # magnetometer in-run calibration in body frame (Gauss)
float32[3] mag_calibration_variance
bool mag_calibration_valid
