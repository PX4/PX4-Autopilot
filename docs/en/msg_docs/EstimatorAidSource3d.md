# EstimatorAidSource3d (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorAidSource3d.msg)

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float32[3] observation
float32[3] observation_variance

float32[3] innovation
float32[3] innovation_filtered

float32[3] innovation_variance

float32[3] test_ratio           # normalized innovation squared
float32[3] test_ratio_filtered  # signed filtered test ratio

bool innovation_rejected        # true if the observation has been rejected
bool fused                      # true if the sample was successfully fused

# TOPICS estimator_aid_src_ev_vel estimator_aid_src_gnss_vel estimator_aid_src_gravity estimator_aid_src_mag

```
