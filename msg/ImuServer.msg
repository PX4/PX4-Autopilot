uint64 timestamp          # time since system start (microseconds)

float32 temperature       # temperature in degrees Celsius

uint64[10] ts             # timestamp of each sample

float32[10] accel_x       # acceleration in the FRD board frame X-axis in m/s^2
float32[10] accel_y       # acceleration in the FRD board frame Y-axis in m/s^2
float32[10] accel_z       # acceleration in the FRD board frame Z-axis in m/s^2
float32[10] gyro_x        # angular velocity in the FRD board frame X-axis in rad/s
float32[10] gyro_y        # angular velocity in the FRD board frame Y-axis in rad/s
float32[10] gyro_z        # angular velocity in the FRD board frame Z-axis in rad/s
