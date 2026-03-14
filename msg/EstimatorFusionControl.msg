uint64 timestamp                # time since system start (microseconds)

# effective CTRL param values with runtime overrides (0 = disabled)
uint8[2] gps_intended
uint8 of_intended
uint8 ev_intended
uint8[4] agp_intended
uint8 baro_intended
uint8 rng_intended
uint8 drag_intended
uint8 mag_intended
uint8 aspd_intended

# whether the estimator is actively fusing data from each source
uint8 gps_active
bool of_active
bool ev_active
uint8 agp_active
bool baro_active
bool rng_active
bool drag_active
bool mag_active
bool aspd_active
