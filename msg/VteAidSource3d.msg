# Vision Target Estimator 3D fusion aid source

uint64 timestamp                 # [us] time since system start
uint64 timestamp_sample          # [us] timestamp of the raw observation
uint64 time_last_predict         # [us] timestamp of last filter prediction

# Observation & Innovation
float32[3] observation           # [-] [@frame NED] sensor observation attempted to be fused
float32[3] observation_variance  # [-] [@frame NED] variance of the observation attempted to be fused

float32[3] innovation            # [-] [@frame NED] Kalman Filter innovation (y = z - Hx)
float32[3] innovation_variance	 # [-] [@frame NED] Kalman Filter variance of the innovation

float32[3] test_ratio            # [-] normalized innovation squared (NIS)

# 0: STATUS_IDLE
# 1: STATUS_FUSED_CURRENT        # Fused immediately (low latency)
# 2: STATUS_FUSED_OOSM           # Fused via history buffer
# 3: STATUS_REJECT_NIS           # Rejected by Normalized Innovation Squared check
# 4: STATUS_REJECT_COV           # Rejected due to invalid/infinite covariance or numerical error
# 5: STATUS_REJECT_TOO_OLD       # Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample
# 6: STATUS_REJECT_TOO_NEW       # Rejected: timestamp in the future (beyond tolerance)
# 7: STATUS_REJECT_STALE         # Rejected: history was reset due to staleness/discontinuity
# 8: STATUS_REJECT_EMPTY         # Rejected: history buffer not yet populated
uint8[3] fusion_status           # [-] Enum detailing the status code of the sensor fusion

# OOSM Diagnostics (Shared across axes)
float32 time_since_meas_ms       # [ms] (now - timestamp_sample)
uint8 history_steps              # [-] number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_gps_pos_target vte_aid_gps_pos_mission vte_aid_gps_vel_target vte_aid_gps_vel_uav
# TOPICS vte_aid_fiducial_marker
