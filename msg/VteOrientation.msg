# Vision Target Estimator orientation state, exposing the full yaw filter output with covariances for logging and tuning.
#
# Published by: vision_target_estimator (VTEOrientation).
# Subscribed by: logger only. The orientation-related fields consumed elsewhere (precision landing) are exposed on landing_target_pose.

uint64 timestamp # [us] Time since system start

bool orientation_valid # [-] Relative orientation estimate valid

float32 yaw # [rad] [@frame NED] Target yaw angle
float32 cov_yaw # [rad^2] Variance of yaw

float32 yaw_rate # [rad/s] [@frame NED] Target yaw rate
float32 cov_yaw_rate # [(rad/s)^2] Variance of yaw_rate
