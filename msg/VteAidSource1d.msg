uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # the timestamp of the raw data (microseconds)
uint64 time_last_predict      # time the from the last filter prediction (microseconds)

# Observation & Innovation
float32 observation
float32 observation_variance

float32 innovation
float32 innovation_variance

float32 test_ratio            # normalized innovation squared (NIS)

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
uint8 fusion_status

# OOSM Diagnostics
float32 time_since_meas_ms     # [ms] (now - timestamp_sample)
uint8 history_steps           # Number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_ev_yaw
