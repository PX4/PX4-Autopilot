uint64 timestamp                # time since system start (microseconds)

# sensor intended for fusion (enabled via EKF2_SENS_EN AND CTRL param != disabled)
bool[2] gps_intended
bool of_intended
bool ev_intended
bool[4] agp_intended
bool baro_intended
bool rng_intended
bool mag_intended
bool aspd_intended
bool rngbcn_intended

# whether the estimator is actively fusing data from each source
bool[2] gps_active
bool of_active
bool ev_active
bool[4] agp_active
bool baro_active
bool rng_active
bool mag_active
bool aspd_active
bool rngbcn_active
