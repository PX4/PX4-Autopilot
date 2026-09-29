# Per-receiver GNSS health and check diagnostics
#
# Published by the sensors module from its per-receiver GnssChecks. Latest-wins: to gate on a sample, use vehicle_gnss.usable.

uint64 timestamp # [us] Time since system start

uint32 device_id_selected # [-] Receiver that vehicle_gnss carries

uint32[4] device_ids                 # [-] One entry per receiver, 0 where unused
bool[4] healthy                      # Passes its checks, and has for long enough (GNSS_REQ_TIME)
uint16[4] failed_checks              # [-] Failed checks among those enabled in GNSS_CHECK, VehicleGnss CHECK_* bits
bool[4] strict                       # Held to GNSS_REQ_*: never passed yet, or disarmed on the ground
float32[4] drift_rate_horizontal     # [m/s] Horizontal position drift rate at rest
float32[4] drift_rate_vertical       # [m/s] Vertical position drift rate at rest
float32[4] speed_horizontal_filtered # [m/s] Filtered horizontal speed at rest

# TOPICS sensors_status_gnss
