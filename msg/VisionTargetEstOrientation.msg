# Relative orientation of the target estimated by the vision target estimator

uint64 timestamp		    # time since system start (microseconds)

bool orientation_valid  # Flag showing whether relative orientation is valid

float32 theta           # Orientation of the target relative to navigation frame [rad] [-pi; pi]
float32 cov_theta       # orientation uncertainty

float32 v_theta         # orientation speed of the target relative to navigation frame [rad/s]
float32 cov_v_theta     # orientation speed uncertainty
