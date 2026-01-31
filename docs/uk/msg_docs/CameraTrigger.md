# CameraTrigger (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CameraTrigger.msg)

```c
uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_utc # UTC timestamp

uint32 seq		# Image sequence number
bool feedback	# Trigger feedback from camera

uint32 ORB_QUEUE_LENGTH = 2

```
