# Ranging beacon measurement data (e.g. LoRa, UWB)

uint64 timestamp                # [us] time since system start
uint64 timestamp_sample         # [us] the timestamp of the raw data
uint8 beacon_id
float32 range                   # [m] Range measurement

float64 lat                     # [deg] Latitude
float64 lon                     # [deg] Longitude
float32 alt                     # [m] Beacon altitude (frame defined in alt_type)
uint8 alt_type                  # [@enum ALT_TYPE] Altitude frame for alt field
uint8 ALT_TYPE_WGS84 = 0        # Altitude above WGS84 ellipsoid
uint8 ALT_TYPE_MSL   = 1        # Altitude above Mean Sea Level (AMSL)

float32 hacc                    # [m] Groundbeacon horizontal accuracy
float32 vacc                    # [m] Groundbeacon vertical accuracy

uint8 sequence_nr
uint8 status
uint16 carrier_freq             # [MHz] Carrier frequency
float32 range_accuracy          # [m] Range accuracy estimate


# TOPICS ranging_beacon
