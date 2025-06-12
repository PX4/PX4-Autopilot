# CollisionReport (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CollisionReport.msg)

```c
uint64 timestamp		# time since system start (microseconds)
uint8 src
uint32 id
uint8 action
uint8 threat_level
float32 time_to_minimum_delta
float32 altitude_minimum_delta
float32 horizontal_minimum_delta

```
