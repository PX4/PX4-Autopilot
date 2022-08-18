# SOARING ESTIMATOR WIND ESTIMATE, USED FOR SELECTING THE CORRECT TRAJECTORY

uint64 timestamp			# time since system start (microseconds)

float32 vx                  # maximum wind in x-direction
float32 vy                  # maximum wind in y-direction
float32 bx                  # wind offset in x-direction
float32 by                  # wind offset in y-direction
float32 h                   # vertical position of shear layer in soaring frame
float32 a                   # shear strength

float32 sigma_vx            # covariance of vx
float32 sigma_vy            # covariance of vy
float32 sigma_bx            # covariance of bx
float32 sigma_by            # covariance of by
float32 sigma_h             # covariance of h
float32 sigma_a             # covariance of a

bool params_healthy         # plausibility check
uint64 reset_counter        # filter reset counter
