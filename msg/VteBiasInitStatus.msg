# Diagnostics for the initial GNSS/vision bias averaging phase in the Vision Target Estimator.
#
# Published by: vision_target_estimator (VTEPosition) while the bias low-pass filter is running.
# Subscribed by: logger only, to verify that the bias settles before the estimator starts fusing vision.

uint64 timestamp # [us] Time since system start

float32[3] raw_bias # [m] [@frame NED] Current GNSS-vision bias sample
float32[3] filtered_bias # [m] [@frame NED] Low-pass filtered bias sample
float32 delta_norm # [m] norm(raw_bias_k - raw_bias_k-1)
