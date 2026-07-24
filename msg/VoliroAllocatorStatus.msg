# Diagnostics for the dedicated six-axis Voliro variable-tilt allocator.
uint64 timestamp                         # [us] Time since system start
uint64 timestamp_sample                  # [us] Sampling timestamp of the torque setpoint

uint8 FALLBACK_NONE = 0
uint8 FALLBACK_DISARMED = 1
uint8 FALLBACK_FEEDBACK_STALE = 2
uint8 FALLBACK_SOLVER_FAILED = 3
uint8 FALLBACK_INVALID_SETPOINT = 4
uint8 FALLBACK_LANDED = 5

float32[6] requested_wrench              # [N,N,N,Nm,Nm,Nm] FLU body-frame SI wrench
float32[6] achieved_wrench               # [N,N,N,Nm,Nm,Nm] Estimated wrench at measured tilt
float32[6] residual_wrench               # [N,N,N,Nm,Nm,Nm] achieved minus requested
float32[6] measured_tilt                  # [rad] Measured radial-axis joint angles
float32[6] commanded_tilt                 # [rad] Rate- and angle-limited joint commands
float32[6] commanded_thrust               # [N] Per-rotor thrust command
float32 residual_norm
float32 solver_time_us                    # [us] Allocation execution time
float32 feedback_age_s                    # [s] Age of latest measured-angle sample
uint16 iterations
uint8 thrust_saturation_mask              # One bit per saturated rotor
uint8 tilt_rate_limited_mask              # One bit per rate-limited tilt command
uint8 fallback_reason                     # FALLBACK_* value
bool optimization_used
bool solver_success
bool feedback_valid
