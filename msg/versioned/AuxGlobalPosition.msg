# Auxiliary global position

# This message provides global position data from an external source such as
# pseudolites, viusal navigation, or other positioning system.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                # [us] Time since system start
uint64 timestamp_sample         # [us] Timestamp of the raw data

uint8 id                        # Unique identifier for the AGP source
uint8 source			# 0: Unkown, 1: GNSS, 2: Vision, 3: Pseudolites, 4: Terrain, 5: Magnetic, 6: Estimator

float64 lat                     # [deg] Latitude in WGS84
float64 lon                     # [deg] Longitude in WGS84
float32 alt                     # [m] Altitude above mean sea level (AMSL)

float32 eph                     # [m] Standard deviation of horizontal position error
float32 epv                     # [m] Standard deviation of vertical position error

uint8 lat_lon_reset_counter     # Counter for reset events on horizontal position coordinates

# TOPICS aux_global_position
