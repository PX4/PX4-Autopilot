uint32 icao_address 	# ICAO address
float64 lat 		# Latitude, expressed as degrees
float64 lon 		# Longitude, expressed as degrees
uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
float32 altitude 	# Altitude(ASL) in meters
float32 heading 	# Course over ground in radians, -pi to +pi, 0 is north
float32 hor_velocity	# The horizontal velocity in m/s
float32 ver_velocity 	# The vertical velocity in m/s, positive is up
char[9] callsign	# The callsign, 8+null
uint8 emitter_type 	# Type from ADSB_EMITTER_TYPE enum
uint8 tslc 		# Time since last communication in seconds
uint16 flags 		# Flags to indicate various statuses including valid data fields
uint16 squawk 		# Squawk code

# ADSB flags
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_RETRANSLATE = 256

uint32 ORB_QUEUE_LENGTH = 10
