uint64 timestamp                          # time since system start (microseconds)
uint64 timestamp_sample                   # the timestamp of the raw data (microseconds)

bool checks_passed

bool check_fail_gps_fix          # insufficient fix type (no 3D solution)
bool check_fail_min_sat_count    # minimum required sat count fail
bool check_fail_max_pdop         # maximum allowed PDOP fail
bool check_fail_max_horz_err     # maximum allowed horizontal position error fail
bool check_fail_max_vert_err     # maximum allowed vertical position error fail
bool check_fail_max_spd_err      # maximum allowed speed error fail
bool check_fail_max_horz_drift   # maximum allowed horizontal position drift fail - requires stationary vehicle
bool check_fail_max_vert_drift   # maximum allowed vertical position drift fail - requires stationary vehicle
bool check_fail_max_horz_spd_err # maximum allowed horizontal speed fail - requires stationary vehicle
bool check_fail_max_vert_spd_err # maximum allowed vertical velocity discrepancy fail
bool check_fail_spoofed_gps      # GPS signal is spoofed

float32 position_drift_rate_horizontal_m_s # Horizontal position rate magnitude (m/s)
float32 position_drift_rate_vertical_m_s   # Vertical position rate magnitude (m/s)
float32 filtered_horizontal_speed_m_s      # Filtered horizontal velocity magnitude (m/s)
