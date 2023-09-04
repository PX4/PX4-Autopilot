# Relative orientation of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		    # time since system start (microseconds)

bool orientation_valid  # Flag showing whether relative position is valid

float32 theta           # Orientation of the target relative to navigation frame [rad] [0 ; 2Pi]
float32 cov_theta       # orientation uncertainty

float32 v_theta         # orientation speed of the target relative to navigation frame [rad/s]
float32 cov_v_theta     # orientation speed uncertainty
