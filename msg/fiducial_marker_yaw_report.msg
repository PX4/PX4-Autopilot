# Relative orientation of landing target in body frame (FRD)

uint64 timestamp		    # time since system start (microseconds)

bool orientation_valid  # Flag showing whether relative position is valid

float32 theta_rel_body       # orientation of the target relative to body frame [rad] [0 ; 2Pi]
float32 cov_theta_rel_body   # orientation uncertainty

float32[4] q        # Quaternion rotation from the FRD body frame to the NED earth frame
