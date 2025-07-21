# ArmingCheckRequest (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ArmingCheckRequest.msg)

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# broadcast message to request all registered arming checks to be reported

uint8 request_id

```
