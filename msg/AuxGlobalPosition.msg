# Auxiliary global position

# This message provides global position data from an external source such as
# radio-triangulation, viusal navigation, or other positioning system.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                # [us] Time since system start
uint64 timestamp_sample         # [us] Timestamp of the raw data

uint8 id                        # Unique identifier for the AGP sourcxe, 1X for Visual Navigation, 2X for Radio Triangulation

float64 lat                     # [deg] Latitude in WGS84
float64 lon                     # [deg] Longitude in WGS84
float32 alt                     # [m] Altitude above mean sea level (AMSL)

float32 eph                     # [m] Standard deviation of horizontal position error
float32 epv                     # [m] Standard deviation of vertical position error

uint8 lat_lon_reset_counter     # Counter for reset events on horizontal position coordinates

# TOPICS aux_global_position
