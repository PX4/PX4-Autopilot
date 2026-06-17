uint64 timestamp # [us] ime since system start
uint64 timestamp_sample # [us] Time of baro data last used for this estimate

float32 residual # [m] CF residual: baro minus accel-predicted altitude
float32 k_estimate # [m/unit_thrust] estimated thrust-to-baro gain
float32 k_estimate_var # [-] variance of K estimate (P[0][0])
float32 error_var # [-] RLS prediction error variance
float32 thrust_std # [-] Standard deviation of recent thrust excitation

bool converged # True when all convergence criteria are met
bool estimation_active # True when RLS is updating (soft guards not active)
