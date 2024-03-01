uint64 timestamp                       # time since system start (microseconds)
uint64 timestamp_sample                # the timestamp of the raw data (microseconds)

float32[2] vel_body                    # velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)
float32[2] vel_ne                      # same as vel_body but in local frame (m/s)

float32[2] flow_rate_uncompensated     # integrated optical flow measurement (rad/s)
float32[2] flow_rate_compensated       # integrated optical flow measurement compensated for angular motion (rad/s)

float32[3] gyro_rate                   # gyro measurement synchronized with flow measurements (rad/s)

float32[3] gyro_bias
float32[3] ref_gyro

# TOPICS estimator_optical_flow_vel vehicle_optical_flow_vel
