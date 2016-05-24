uint32 ICAO_address 	# ICAO address
float64 lat 		# Latitude, expressed as degrees
float64 lon 		# Longitude, expressed as degrees
uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
float32 altitude 	# Altitude(ASL) in meters
float32 heading 	# Course over ground in radians
float32 hor_velocity	# The horizontal velocity in m/s
float32 ver_velocity 	# The vertical velocity in m/s, positive is up
char[9] callsign	# The callsign, 8+null
uint8 emitter_type 	# Type from ADSB_EMITTER_TYPE enum
uint8 tslc 		# Time since last communication in seconds
uint16 flags 		# Flags to indicate various statuses including valid data fields
uint16 squawk 		# Squawk code
