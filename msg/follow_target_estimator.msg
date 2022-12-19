uint64 timestamp                     # time since system start (microseconds)
uint64 last_filter_reset_timestamp   # time of last filter reset (microseconds)

bool valid              # True if estimator states are okay to be used
bool stale              # True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate.

float64 lat_est         # Estimated target latitude
float64 lon_est         # Estimated target longitude
float32 alt_est         # Estimated target altitude

float32[3] pos_est      # Estimated target NED position (m)
float32[3] vel_est      # Estimated target NED velocity (m/s)
float32[3] acc_est      # Estimated target NED acceleration (m^2/s)
