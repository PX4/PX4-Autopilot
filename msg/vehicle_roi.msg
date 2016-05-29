# Vehicle Region Of Interest (ROI)

uint8 VEHICLE_ROI_NONE = 0                         # No region of interest |
uint8 VEHICLE_ROI_WPNEXT = 1                       # Point toward next MISSION |
uint8 VEHICLE_ROI_WPINDEX = 2                      # Point toward given MISSION |
uint8 VEHICLE_ROI_LOCATION = 3                     # Point toward fixed location |
uint8 VEHICLE_ROI_TARGET = 4                       # Point toward target
uint8 VEHICLE_ROI_ENUM_END = 5

uint8 mode          # ROI mode (see above)
uint32 mission_seq  # mission sequence to point to
uint32 target_seq   # target sequence to point to
float64 lat			    # Latitude to point to
float64 lon			    # Longitude to point to
float32 alt			    # Altitude to point to
