uint64 timestamp             # time since system start (microseconds)
uint64 timestamp_sample      # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64[3] time_last_fuse

float32[3] observation
float32[3] observation_variance

float32[3] innovation
float32[3] innovation_variance
float32[3] test_ratio

bool[3] fusion_enabled       # true when measurements are being fused
bool[3] innovation_rejected  # true if the observation has been rejected
bool[3] fused                # true if the sample was successfully fused

# TOPICS estimator_aid_source_3d
# TOPICS estimator_aid_src_gnss_pos estimator_aid_src_gnss_vel
