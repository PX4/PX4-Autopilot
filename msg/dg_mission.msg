
uint64 timestamp	# time since system start (microseconds)


uint8 DG_MISSION_MSG = 0 

uint8 DG_MISSION_COUNT_MSG = 1

uint8 DG_MISSION_SET_CURRENT = 2

uint8 DG_MISSION_CLEAR_ALL_MSG = 3


uint8  msg_type


uint16 count 		# Number of mission items in the sequence


uint8 target_system 	# System ID

uint8 target_component 	# Component ID


uint8 mission_type      # Mission type / MAV_MISSION_TYPE_MISSION=0

float32 param1 # PARAM1, see MAV_CMD enum

float32 param2 # PARAM2, see MAV_CMD enum

float32 param3 # PARAM3, see MAV_CMD enum

float32 param4 # PARAM4, see MAV_CMD enum

float32 x # PARAM5 / local: X coordinate, global: latitude

float32 y # PARAM6 / local: Y coordinate, global: longitude

float32 z # PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame)

uint16 seq   # Sequence

uint16 command # The scheduled action for the waypoint

uint8 frame  # The coordinate system of the waypoint

uint8 current # false:0, true:1

uint8 autocontinue  # Autocontinue to next waypoint

float32 cruise_speed  # dg cruise speed