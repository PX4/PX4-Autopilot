# SOARING ESTIMATOR WIND ESTIMATE, USED FOR SELECTING THE CORRECT TRAJECTORY

uint64 timestamp			# time since system start (microseconds)

float32 vx                  # maximum wind in x-direction
float32 vy                  # maximum wind in y-direction
float32 bx                  # wind offset in x-direction
float32 by                  # wind offset in y-direction
float32 h                   # vertical position of shear layer in soaring frame
float32 a                   # shear strength

bool params_healthy         # plausibility check
uint64 reset_counter        # filter reset counter
