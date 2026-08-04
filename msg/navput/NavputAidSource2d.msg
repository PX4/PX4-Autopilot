uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float64[2] observation
float32[2] observation_variance

float32[2] innovation
float32[2] innovation_filtered

float32[2] innovation_variance

float32[2] test_ratio           # normalized innovation squared
float32[2] test_ratio_filtered  # signed filtered test ratio

bool innovation_rejected        # true if the observation has been rejected
bool fused                      # true if the sample was successfully fused

# TOPICS navput_aid_src_aux_global_position
