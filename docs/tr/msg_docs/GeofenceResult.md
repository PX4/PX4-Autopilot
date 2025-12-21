# GeofenceResult (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeofenceResult.msg)

```c
uint64 timestamp                        # time since system start (microseconds)
uint8 GF_ACTION_NONE = 0                # no action on geofence violation
uint8 GF_ACTION_WARN = 1                # critical mavlink message
uint8 GF_ACTION_LOITER = 2              # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                 # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4           # flight termination
uint8 GF_ACTION_LAND = 5                # switch to AUTO|LAND

bool geofence_max_dist_triggered	# true the check for max distance from Home is triggered
bool geofence_max_alt_triggered		# true the check for max altitude above Home is triggered
bool geofence_custom_fence_triggered	# true the check for custom inclusion/exclusion geofence(s) is triggered

uint8 geofence_action           	# action to take when the geofence is breached

```
