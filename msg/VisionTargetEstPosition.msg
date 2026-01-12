# Vision Target Estimator position state

uint64 timestamp           # [us] time since system start

bool rel_pos_valid         # [-] relative position estimate valid
bool rel_vel_valid         # [-] relative velocity estimate valid

float32[3] rel_pos         # [m] [@frame NED] target position relative to vehicle
float32[3] vel_uav         # [m/s] [@frame NED] vehicle velocity
float32[3] vel_target      # [m/s] [@frame NED] target velocity
float32[3] bias            # [m] [@frame NED] GNSS bias between vehicle and target receivers
float32[3] acc_target      # [m/s^2] [@frame NED] target acceleration

float32[3] cov_rel_pos     # [m^2] [@frame NED] variance of rel_pos
float32[3] cov_vel_uav     # [(m/s)^2] [@frame NED] variance of vel_uav
float32[3] cov_bias        # [m^2] [@frame NED] variance of bias
float32[3] cov_vel_target  # [(m/s)^2] [@frame NED] variance of vel_target
float32[3] cov_acc_target  # [(m/s^2)^2] [@frame NED] variance of acc_target
