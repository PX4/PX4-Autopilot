# Auxiliary global position
#
# This message provides global position data from an external source such as
# pseudolites, visual navigation, or other positioning system.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                # [us] Time since system start
uint64 timestamp_sample         # [us] Timestamp of the raw data

uint8 id                        # [-] Unique identifier for the AGP source
uint8 source                    # [@enum SOURCE] Source type of the position data (based on mavlink::GLOBAL_POSITION_SRC)
uint8 SOURCE_UNKNOWN = 0        # Unknown source
uint8 SOURCE_GNSS = 1           # GNSS
uint8 SOURCE_VISION = 2         # Vision
uint8 SOURCE_PSEUDOLITES = 3    # Pseudolites
uint8 SOURCE_TERRAIN = 4        # Terrain
uint8 SOURCE_MAGNETIC = 5       # Magnetic
uint8 SOURCE_ESTIMATOR = 6      # Estimator

# lat, lon: required for horizontal position fusion, alt: required for vertical position fusion
float64 lat                     # [deg] Latitude in WGS84
float64 lon                     # [deg] Longitude in WGS84
float32 alt                     # [m] [@invalid NaN] Altitude above mean sea level (AMSL)

float32 eph                     # [m] [@invalid NaN] Std dev of horizontal position, lower bounded by NOISE param
float32 epv                     # [m] [@invalid NaN] Std dev of vertical position, lower bounded by NOISE param

uint8 lat_lon_reset_counter     # [-] Counter for reset events on horizontal position coordinates

# TOPICS aux_global_position
