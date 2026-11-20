uint64 timestamp                        # time since system start (microseconds)
uint8 GF_ACTION_NONE = 0                # no action on geofence violation
uint8 GF_ACTION_WARN = 1                # critical mavlink message
uint8 GF_ACTION_LOITER = 2              # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                 # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4           # flight termination
uint8 GF_ACTION_LAND = 5                # switch to AUTO|LAND

bool geofence_emer_max_dist_triggered	# true if the emergency geofence check for max distance from Home is triggered
bool geofence_emer_max_alt_triggered	# true if the emergency geofence check for max altitude above Home is triggered
bool geofence_emer_custom_fence_triggered	# true if the emergency geofence check for custom inclusion/exclusion geofence(s) is triggered
bool geofence_cont_max_dist_triggered	# true if the contingency geofence check for max distance from Home is triggered
bool geofence_cont_custom_fence_triggered	# true if the contingency geofence check for custom inclusion/exclusion geofence(s) is triggered

uint8 geofence_emer_action       	# action to take when the emergency geofence is breached
uint8 geofence_cont_action       	# action to take when the contingency geofence is breached, GF_ACTION_NONE while it is disabled
