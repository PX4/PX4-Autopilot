# GeofenceStatus (повідомлення UORB)

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GeofenceStatus.msg)

```c
uint64 timestamp                        # time since system start (microseconds)

uint32 geofence_id 			# loaded geofence id
uint8 status 				# Current geofence status

uint8 GF_STATUS_LOADING = 0
uint8 GF_STATUS_READY = 1

```
