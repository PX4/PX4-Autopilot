uint64 timestamp          # time since system start (microseconds)

float32[3] gyro_angles    # Roll, pitch, yaw from gyro integration
float32[3] acc_angles     # Roll, pitch, yaw from accelerometer
float32[3] comp_angles    # Final fused angles