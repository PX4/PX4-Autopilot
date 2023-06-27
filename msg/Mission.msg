uint64 timestamp	# time since system start (microseconds)
uint8 dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1

uint16 count		# count of the missions stored in the dataman
int32 current_seq	# default -1, start at the one changed latest

int32 land_start_index  # Index of the land start marker, if unavailable index of the land item, -1 otherwise
int32 land_index 	# Index of the land item, -1 otherwise

uint16 mission_update_counter # indicates updates to the mission, reload from dataman if increased
uint16 geofence_update_counter # indicates updates to the geofence, reload from dataman if increased
uint16 safe_points_update_counter # indicates updates to the safe points, reload from dataman if increased
