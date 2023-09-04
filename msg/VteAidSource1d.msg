# Vision Target Estimator 1D fusion aid-source diagnostics (e.g. yaw).
#
# Published by: vision_target_estimator (VTEOrientation) on every fusion attempt, including rejected ones.
# Subscribed by: logger only. Inspect observation, innovation, test_ratio, and fusion_status to debug why a measurement was or was not fused.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation
uint64 time_last_predict # [us] Timestamp of last filter prediction

# Observation & Innovation
float32 observation # [-] Observation attempted to be fused
float32 observation_variance # [-] Variance of observation attempted to be fused

float32 innovation # [-] Kalman Filter innovation (y = z - Hx)
float32 innovation_variance # [-] Kalman Filter variance of the innovation

float32 test_ratio # [-] Normalized innovation squared (NIS)

uint8 fusion_status # [@enum VTE_FUSION_STATUS] Fusion status code
uint8 STATUS_IDLE = 0 # No fusion attempted yet
uint8 STATUS_FUSED_CURRENT = 1 # Fused immediately (low latency)
uint8 STATUS_FUSED_OOSM = 2 # Fused via history buffer
uint8 STATUS_REJECT_NIS = 3 # Rejected by Normalized Innovation Squared check
uint8 STATUS_REJECT_COV = 4 # Rejected due to invalid/infinite covariance or numerical error
uint8 STATUS_REJECT_TOO_OLD = 5 # Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample
uint8 STATUS_REJECT_TOO_NEW = 6 # Rejected: timestamp in the future (beyond tolerance)
uint8 STATUS_REJECT_STALE = 7 # Rejected: history was reset due to staleness/discontinuity
uint8 STATUS_REJECT_EMPTY = 8 # Rejected: history buffer not yet populated

# OOSM Diagnostics
float32 time_since_meas_ms # [ms] (now - timestamp_sample)
uint8 history_steps # [-] Number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_ev_yaw
