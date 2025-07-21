# DistanceSensorModeChangeRequest (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DistanceSensorModeChangeRequest.msg)

```c
uint64 timestamp		# time since system start (microseconds)

uint8 request_on_off 			# request to disable/enable the distance sensor
uint8 REQUEST_OFF = 0
uint8 REQUEST_ON  = 1

```
