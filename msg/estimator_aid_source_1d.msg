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

bool fusion_enabled          # true when measurements are being fused
bool innovation_rejected     # true if the observation has been rejected
bool fused                   # true if the sample was successfully fused

# TOPICS estimator_aid_source_1d
# TOPICS estimator_aid_src_baro_hgt estimator_aid_src_rng_hgt
