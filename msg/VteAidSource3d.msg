# Vision Target Estimator 3D fusion aid-source diagnostics, one fusion_status per NED axis.
#
# Published by: vision_target_estimator (VTEPosition) on every fusion attempt, including rejected ones.
# Subscribed by: logger only. Inspect observation, innovation, test_ratio, and per-axis fusion_status to debug why a measurement was or was not fused.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation
uint64 time_last_predict # [us] Timestamp of last filter prediction

# Observation & Innovation
float32[3] observation # [-] [@frame NED] Sensor observation attempted to be fused
float32[3] observation_variance # [-] [@frame NED] Variance of the observation attempted to be fused

float32[3] innovation # [-] [@frame NED] Kalman Filter innovation (y = z - Hx)
float32[3] innovation_variance # [-] [@frame NED] Kalman Filter variance of the innovation

float32[3] test_ratio # [-] Normalized innovation squared (NIS)

uint8[3] fusion_status # [@enum VTE_FUSION_STATUS] Fusion status code per axis
uint8 STATUS_IDLE = 0 # No fusion attempted yet
uint8 STATUS_FUSED_CURRENT = 1 # Fused immediately (low latency)
uint8 STATUS_FUSED_OOSM = 2 # Fused via history buffer
uint8 STATUS_REJECT_NIS = 3 # Rejected by Normalized Innovation Squared check
uint8 STATUS_REJECT_COV = 4 # Rejected due to invalid/infinite covariance or numerical error
uint8 STATUS_REJECT_TOO_OLD = 5 # Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample
uint8 STATUS_REJECT_TOO_NEW = 6 # Rejected: timestamp in the future (beyond tolerance)
uint8 STATUS_REJECT_STALE = 7 # Rejected: history was reset due to staleness/discontinuity
uint8 STATUS_REJECT_EMPTY = 8 # Rejected: history buffer not yet populated

# OOSM Diagnostics (Shared across axes)
float32 time_since_meas_ms # [ms] (now - timestamp_sample)
uint8 history_steps # [-] Number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_gps_pos_target vte_aid_gps_pos_mission vte_aid_gps_vel_target vte_aid_gps_vel_uav
# TOPICS vte_aid_fiducial_marker
