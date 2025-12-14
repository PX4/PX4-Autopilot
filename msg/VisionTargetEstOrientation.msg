# Relative orientation of the target estimated by the vision target estimator

uint64 timestamp		    # time since system start (microseconds)

bool orientation_valid  # Flag showing whether relative orientation is valid

float32 yaw            # yaw of the target relative to the navigation frame [rad] [-pi; pi]
float32 cov_yaw        # yaw variance [rad^2]

float32 yaw_rate       # yaw rate of the target relative to the navigation frame [rad/s]
float32 cov_yaw_rate   # yaw rate variance [(rad/s)^2]
