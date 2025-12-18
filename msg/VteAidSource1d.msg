# Vision Target Estimator 1D fusion aid source

uint64 timestamp              # [us] time since system start
uint64 timestamp_sample       # [us] timestamp of the raw observation
uint64 time_last_predict      # [us] timestamp of last filter prediction

# Observation & Innovation
float32 observation           # [-] observation attempted to be fused
float32 observation_variance  # [-] variance of observation attempted to be fused

float32 innovation            # [-] Kalman Filter innovation (y = z - Hx)
float32 innovation_variance   # [-] Kalman Filter variance of the innovation

float32 test_ratio            # [-] normalized innovation squared (NIS)

# Detailed status code for debugging "Why did it fail?"
# 0: STATUS_IDLE
# 1: STATUS_FUSED_CURRENT     # Fused immediately (low latency)
# 2: STATUS_FUSED_OOSM        # Fused via history buffer
# 3: STATUS_REJECT_NIS        # Rejected by Normalized Innovation Squared check
# 4: STATUS_REJECT_COV        # Rejected due to invalid/infinite covariance or numerical error
# 5: STATUS_REJECT_TOO_OLD    # Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample
# 6: STATUS_REJECT_TOO_NEW    # Rejected: timestamp in the future (beyond tolerance)
# 7: STATUS_REJECT_STALE      # Rejected: history was reset due to staleness/discontinuity
# 8: STATUS_REJECT_EMPTY      # Rejected: history buffer not yet populated
uint8 fusion_status           # [-] fusion status code

# OOSM Diagnostics
float32 time_since_meas_ms     # [ms] (now - timestamp_sample)
uint8 history_steps            # [-] number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_ev_yaw
