# Vision Target Estimator position state, exposing the full per-axis Kalman filter state with covariances for logging and tuning.
#
# Published by: vision_target_estimator (VTEPosition).
# Subscribed by: logger only. The position-related fields consumed elsewhere (precision landing, EKF2 aiding) are exposed on landing_target_pose.
#
# vel_target and acc_target are only populated when the firmware is built with CONFIG_VTEST_MOVING=y; otherwise they stay at zero.

uint64 timestamp # [us] Time since system start

bool rel_pos_valid # [-] Relative position estimate valid
bool rel_vel_valid # [-] Relative velocity estimate valid

float32[3] rel_pos # [m] [@frame NED] Target position relative to vehicle
float32[3] vel_uav # [m/s] [@frame NED] Vehicle velocity
float32[3] vel_target # [m/s] [@frame NED] Target velocity
float32[3] bias # [m] [@frame NED] GNSS bias between vehicle and target receivers
float32[3] acc_target # [m/s^2] [@frame NED] Target acceleration

float32[3] cov_rel_pos # [m^2] [@frame NED] Variance of rel_pos
float32[3] cov_vel_uav # [(m/s)^2] [@frame NED] Variance of vel_uav
float32[3] cov_bias # [m^2] [@frame NED] Variance of bias
float32[3] cov_vel_target # [(m/s)^2] [@frame NED] Variance of vel_target
float32[3] cov_acc_target # [(m/s^2)^2] [@frame NED] Variance of acc_target
