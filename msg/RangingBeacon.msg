# Ranging beacon measurement data (e.g. LoRa, UWB)

uint64 timestamp                # [us] time since system start
uint64 timestamp_sample         # [us] the timestamp of the raw data
uint8 beacon_id
float32 range                   # [m] Range measurement

float64 lat                     # [deg] Latitude
float64 lon                     # [deg] Longitude
float32 alt                     # [m] Altitude AMSL
float32 alt_ellipsoid           # [m] Altitude above ellipsoid

float32 hacc                    # [m] Groundbeacon horizontal accuracy
float32 vacc                    # [m] Groundbeacon vertical accuracy

int8 rssi_node                  # [dBm] RSSI at node/vehicle side
int8 rssi_beacon                # [dBm] RSSI at beacon/station side
uint8 sequence
uint16 carrier_freq             # [MHz] Carrier frequency
float32 range_accuracy          # [m] Range accuracy estimate


# TOPICS ranging_beacon
