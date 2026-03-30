uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # time of baro data last used for this estimate

float32 residual                # CF residual: baro minus accel-predicted altitude [m]
float32 k_estimate              # estimated thrust-to-baro gain from best bank [m/unit_thrust]
float32 k_estimate_var          # variance of K estimate (P[0][0] of best bank)
float32 best_tau                # time constant of best-performing RLS bank [s]
float32 best_error_var          # prediction error variance of best bank
float32 thrust_std              # standard deviation of recent thrust excitation

bool converged                  # true when all convergence criteria are met
bool estimation_active          # true when RLS is updating (soft guards not active)
uint8 best_bank_idx             # index of best-performing RLS bank
