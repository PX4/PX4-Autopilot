# Vision Target Estimator orientation state

uint64 timestamp		    # [us] time since system start

bool orientation_valid  # [-] relative orientation estimate valid

float32 yaw            # [rad] [@frame NED] target yaw angle
float32 cov_yaw        # [rad^2] variance of yaw

float32 yaw_rate       # [rad/s] [@frame NED] target yaw rate
float32 cov_yaw_rate   # [(rad/s)^2] variance of yaw_rate
