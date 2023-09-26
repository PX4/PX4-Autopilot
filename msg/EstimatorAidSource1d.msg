uint64 timestamp             # time since system start (microseconds)
uint64 timestamp_sample      # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float32 observation
float32 observation_variance

float32 innovation
float32 innovation_variance
float32 test_ratio

bool innovation_rejected     # true if the observation has been rejected
bool fused                   # true if the sample was successfully fused

# TOPICS estimator_aid_src_baro_hgt estimator_aid_src_ev_hgt estimator_aid_src_gnss_hgt estimator_aid_src_rng_hgt
# TOPICS estimator_aid_src_airspeed estimator_aid_src_sideslip
# TOPICS estimator_aid_src_fake_hgt
# TOPICS estimator_aid_src_mag_heading estimator_aid_src_gnss_yaw estimator_aid_src_ev_yaw
# TOPICS estimator_aid_src_terrain_range_finder
