# Місія (Повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Mission.msg)

```c
uint64 timestamp	# time since system start (microseconds)
uint8 mission_dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1
uint8 fence_dataman_id		# default 0, there are two offboard storage places in the dataman: 0 or 1
uint8 safepoint_dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1

uint16 count		# count of the missions stored in the dataman
int32 current_seq	# default -1, start at the one changed latest

int32 land_start_index  # Index of the land start marker, if unavailable index of the land item, -1 otherwise
int32 land_index 	# Index of the land item, -1 otherwise

uint32 mission_id # indicates updates to the mission, reload from dataman if changed
uint32 geofence_id # indicates updates to the geofence, reload from dataman if changed
uint32 safe_points_id # indicates updates to the safe points, reload from dataman if changed

```
