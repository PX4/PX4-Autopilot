uint8 VEHICLE_MOUNT_MODE_RETRACT = 0				# Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization |
uint8 VEHICLE_MOUNT_MODE_NEUTRAL = 1				# Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. |
uint8 VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2			# Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization |
uint8 VEHICLE_MOUNT_MODE_RC_TARGETING = 3			# Load neutral position and start RC Roll,Pitch,Yaw control with stabilization |
uint8 VEHICLE_MOUNT_MODE_GPS_POINT = 4				# Load neutral position and start to point to Lat,Lon,Alt |
uint8 VEHICLE_MOUNT_MODE_ENUM_END = 5				#


uint8 mode                                          # The current mount mode
bool stab_roll                                      # stabilize roll? (1 = yes, 0 = no)
bool stab_pitch                                     # stabilize pitch? (1 = yes, 0 = no)
bool stab_way                                       # stabilize yaw? (1 = yes, 0 = no)

float64 pitch_lat                                   # pitch or lat in degrees, depending on mount mode
float64 roll_lon                                    # roll or lon in degrees depending on mount mode
float32 yaw_alt                                     # yaw or alt (in meters) depending on mount mode
