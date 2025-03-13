# EstimatorGpsStatus (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorGpsStatus.msg)

```c
uint64 timestamp                          # time since system start (microseconds)
uint64 timestamp_sample                   # the timestamp of the raw data (microseconds)

bool checks_passed

bool check_fail_gps_fix          # 0 : insufficient fix type (no 3D solution)
bool check_fail_min_sat_count    # 1 : minimum required sat count fail
bool check_fail_max_pdop         # 2 : maximum allowed PDOP fail
bool check_fail_max_horz_err     # 3 : maximum allowed horizontal position error fail
bool check_fail_max_vert_err     # 4 : maximum allowed vertical position error fail
bool check_fail_max_spd_err      # 5 : maximum allowed speed error fail
bool check_fail_max_horz_drift   # 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
bool check_fail_max_vert_drift   # 7 : maximum allowed vertical position drift fail - requires stationary vehicle
bool check_fail_max_horz_spd_err # 8 : maximum allowed horizontal speed fail - requires stationary vehicle
bool check_fail_max_vert_spd_err # 9 : maximum allowed vertical velocity discrepancy fail
bool check_fail_spoofed_gps      # 10 : GPS signal is spoofed

float32 position_drift_rate_horizontal_m_s # Horizontal position rate magnitude (m/s)
float32 position_drift_rate_vertical_m_s   # Vertical position rate magnitude (m/s)
float32 filtered_horizontal_speed_m_s      # Filtered horizontal velocity magnitude (m/s)

```
