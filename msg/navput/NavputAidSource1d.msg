uint64 timestamp             # time since system start (microseconds)
uint64 timestamp_sample      # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float32 observation
float32 observation_variance

float32 innovation
float32 innovation_filtered

float32 innovation_variance

float32 test_ratio           # normalized innovation squared
float32 test_ratio_filtered  # signed filtered test ratio

bool innovation_rejected     # true if the observation has been rejected
bool fused                   # true if the sample was successfully fused

# TOPICS navput_aid_src_baro_hgt navput_aid_src_ranging_beacon
