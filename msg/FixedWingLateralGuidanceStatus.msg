# Fixed Wing Lateral Guidance Status message
# Published by fw_pos_control module to report the resultant lateral setpoints and NPFG debug outputs

uint64 timestamp                # time since system start (microseconds)

float32 course_setpoint         # [rad] [@range -pi, pi] Desired direction of travel over ground w.r.t (true) North. Set by guidance law
float32 lateral_acceleration_ff # [m/s^2] [FRD] lateral acceleration demand only for maintaining curvature
float32 bearing_feas            # [@range 0,1] bearing feasibility
float32 bearing_feas_on_track   # [@range 0,1] on-track bearing feasibility
float32 signed_track_error      # [m] signed track error
float32 track_error_bound       # [m] track error bound
float32 adapted_period          # [s] adapted period (if auto-tuning enabled)
uint8 wind_est_valid            # [boolean] true = wind estimate is valid and/or being used by controller (also indicates if wind estimate usage is disabled despite being valid)
