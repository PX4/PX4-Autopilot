# Relative orientation of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp		    # time since system start (microseconds)

bool rel_orientation_valid  # Flag showing whether relative position is valid

float32 theta_rel           # orientation of the target relative to body frame [rad] [-Pi ; Pi]
float32 cov_theta_rel       # orientation uncertainty

float32 v_theta_rel         # orientation speed of the target relative to body frame [rad/s]
float32 cov_v_theta_rel     # orientation speed uncertainty

bool abs_pos_valid		    # Flag showing whether absolute position is valid
float32 theta_abs           # Orientation of the target relative to navigation frame [rad] [-Pi ; Pi]

# TOPICS landing_target_orientation
# TOPICS fiducial_marker_orientation
