# Vehicle Region Of Interest (ROI)

uint8 ROI_NONE = 0                         # No region of interest
uint8 ROI_WPNEXT = 1                       # Point toward next MISSION with optional offset
uint8 ROI_LOCATION = 2                     # Point toward fixed location
uint8 ROI_TARGET = 3                       # Point toward target
uint8 ROI_ENUM_END = 4

uint8 mode          # ROI mode (see above)

float64 lat			    # Latitude to point to
float64 lon			    # Longitude to point to
float32 alt			    # Altitude to point to

float32 pitchOffset		# Additional pitch offset to next waypoint
float32 rollOffset		# Additional roll offset to next waypoint
float32 yawOffset		# Additional yaw offset to next waypoint
