# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)
uint32 	time_uwb_ms		# Time of UWB device in ms
uint32 	counter			# Number of Ranges since last Start of Ranging
uint32 	sessionid		# UWB SessionID
uint32 	time_offset		# Time between Ranging Rounds in ms
uint16 	status			# status feedback #

uint16[12] anchor_distance	# distance in cm to each UWB Anchor (UWB Device wich takes part in Ranging)
bool[12] nlos			# Visual line of sight yes/no
float32[12] aoafirst		# Angle of arrival of first incomming RX msg

float32[3] position		# Position of the Landing point in relation to the Drone (x,y,z in Meters NED)
