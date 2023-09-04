# Vehicle inputs fed into the Vision Target Estimator position prediction step, logged for tuning.
#
# Published by: vision_target_estimator (VisionTargetEst work item).
# Subscribed by: logger only.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw input data

float32[3] acc_xyz # [m/s^2] [@frame NED] Downsampled UAV bias-corrected acceleration (including gravity)
float32[4] q_att # [-] Downsampled UAV attitude quaternion (FRD body -> NED earth)
uint32 acc_sample_count # [-] Number of raw samples averaged into acc_xyz this cycle
