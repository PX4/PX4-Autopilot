uint64 timestamp                # time since system start (microseconds)

float32 course_setpoint         # bearing angle (same as course) [rad]
float32 lateral_acceleration_ff # lateral acceleration demand only for maintaining curvature [m/s^2]
float32 bearing_feas            # bearing feasibility [0,1]
float32 bearing_feas_on_track   # on-track bearing feasibility [0,1]
float32 signed_track_error      # signed track error [m]
float32 track_error_bound       # track error bound [m]
float32 adapted_period          # adapted period (if auto-tuning enabled) [s]
uint8 wind_est_valid            # (boolean) true = wind estimate is valid and/or being used by controller (also indicates if wind est usage is disabled despite being valid)
