# Vision Target Estimator input


uint64 timestamp         # [us] time since system start
uint64 timestamp_sample  # [us] timestamp of the raw input data

float32[3] acc_xyz       # [m/s^2] [@frame NED] downsampled UAV bias-corrected acceleration (including gravity)
float32[4] q_att         # [-] downsampled UAV attitude quaternion (FRD body -> NED earth)
