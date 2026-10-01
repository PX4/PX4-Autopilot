# Per-receiver GNSS health and check diagnostics
#
# Published by the sensors module. Describes the latest sample of each receiver: to gate on a sample, use vehicle_gnss.usable.

uint64 timestamp # [us] Time since system start

uint32 device_id_selected # [-] Receiver that vehicle_gnss carries

uint8 MAX_RECEIVERS = 4 # Length of the per-receiver arrays

uint32[4] device_ids                 # [-] One entry per receiver, 0 where unused
int8[4] order                        # [-] Fixed position of the receiver: 0 for the preferred one, otherwise for the first to publish, then in order of first publication. Changes only with the preference, -1 until the receiver publishes
bool[4] healthy                      # Publishing, and has passed its checks for long enough
float32[4] availability              # [-] Fraction of recent time the receiver passed its checks and published at its usual rate, 0 to 1
float32[4] inconsistency             # [m] [@invalid NaN] Absolute difference between the horizontal distance to the selected receiver's position at the time of its sample and the horizontal distance between their antennas. 0 for the selected receiver
uint16[4] failed_checks              # [-] Bitmask of the enabled checks that failed, VehicleGnss CHECK_* bits
bool[4] strict                       # The pre-flight thresholds apply rather than the relaxed in-flight ones
float32[4] drift_rate_horizontal     # [m/s] Horizontal position drift rate at rest
float32[4] drift_rate_vertical       # [m/s] Vertical position drift rate at rest
float32[4] speed_horizontal_filtered # [m/s] Filtered horizontal speed at rest

# TOPICS sensors_status_gnss
