# Vehicle Region Of Interest (ROI)

uint8 ROI_NONE = 0                         # No region of interest
uint8 ROI_WPNEXT = 1                       # Point toward next MISSION with optional offset
uint8 ROI_WPINDEX = 2                      # Point toward given MISSION
uint8 ROI_LOCATION = 3                     # Point toward fixed location
uint8 ROI_TARGET = 4                       # Point toward target
uint8 ROI_ENUM_END = 5

uint8 mode          # ROI mode (see above)

uint32 mission_seq  # mission sequence to point to
uint32 target_seq   # target sequence to point to

float64 lat			    # Latitude to point to
float64 lon			    # Longitude to point to
float32 alt			    # Altitude to point to

float32 pitchOffset		# Additional pitch offset to next waypoint
float32 rollOffset		# Additional roll offset to next waypoint
float32 yawOffset		# Additional yaw offset to next waypoint
