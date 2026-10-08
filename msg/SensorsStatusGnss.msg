# Per-receiver GNSS health and check diagnostics
#
# Published by the sensors module. Describes the latest sample of each receiver: to gate on a sample, use vehicle_gnss.usable.

uint64 timestamp # [us] Time since system start

uint32 device_id_selected # [-] Receiver that vehicle_gnss carries

uint8 MAX_RECEIVERS = 4 # Length of the per-receiver arrays

uint32[4] device_ids                 # [-] One entry per receiver, 0 where unused
int8[4] order                        # [-] Fixed position of the receiver: 0 for the preferred one, otherwise for the first to publish, then in order of first publication. Changes only with the preference, -1 until the receiver publishes
bool[4] healthy                      # Publishing, and has passed its checks for long enough
uint16[4] failed_checks              # [-] Bitmask of the enabled checks that failed, VehicleGnss CHECK_* bits
bool[4] strict                       # The pre-flight thresholds apply rather than the relaxed in-flight ones
float32[4] drift_rate_horizontal     # [m/s] Horizontal position drift rate at rest
float32[4] drift_rate_vertical       # [m/s] Vertical position drift rate at rest
float32[4] speed_horizontal_filtered # [m/s] Filtered horizontal speed at rest

uint8 heading_state # [@enum HEADING] Dual antenna heading: published, or the furthest a receiver's heading got in the last 3 s, in this order
uint8 HEADING_NONE              = 0 # No receiver reported a valid heading in the last 3 s
uint8 HEADING_UNCONFIGURED      = 1 # Reported, but SENS_GNSSn_HDG is Disabled for the receiver, or no SENS_GNSSn_ID matches it
uint8 HEADING_NO_BASELINE       = 2 # Reported, but the antenna positions SENS_GNSSn_HDG measures between are less than 5 cm apart
uint8 HEADING_BASELINE_MISMATCH = 3 # Reported, but the receiver's baseline length is more than 20% off the antenna positions
uint8 HEADING_SETTLING          = 4 # Reported and matching, published once it has for 1 s
uint8 HEADING_PUBLISHED         = 5 # Published on vehicle_gnss_heading

# TOPICS sensors_status_gnss
