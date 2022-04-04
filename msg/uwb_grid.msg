# UWB report message contains the grid information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp	# time since system start (microseconds)
uint16 	initator_time	# time to synchronize clocks (microseconds)
uint8 	num_anchors	# Number of anchors

float64[4] target_gps	# GPS position of target of the UWB system (Lat / Lon / Alt / Yaw Offset to true North)

# Grid position information
# Position in x,y,z in  (x,y,z in centimeters NED)
# staring with POI and Anchor 0 up to Anchor 11
int16[3] target_pos 	# Point of Interest, mostly landing Target x,y,z
int16[3] anchor_pos_0
int16[3] anchor_pos_1
int16[3] anchor_pos_2
int16[3] anchor_pos_3
int16[3] anchor_pos_4
int16[3] anchor_pos_5
int16[3] anchor_pos_6
int16[3] anchor_pos_7
int16[3] anchor_pos_8
int16[3] anchor_pos_9
int16[3] anchor_pos_10
int16[3] anchor_pos_11
