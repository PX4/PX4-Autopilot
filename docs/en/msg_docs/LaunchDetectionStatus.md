# LaunchDetectionStatus (UORB message)

Status of the launch detection state machine (fixed-wing only)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LaunchDetectionStatus.msg)

```c
# Status of the launch detection state machine (fixed-wing only)

uint64 timestamp # time since system start (microseconds)

uint8 STATE_WAITING_FOR_LAUNCH 			= 0 # waiting for launch
uint8 STATE_LAUNCH_DETECTED_DISABLED_MOTOR 	= 1 # launch detected, but keep motor(s) disabled (e.g. because it can't spin freely while on catapult)
uint8 STATE_FLYING 				= 2 # launch detected, use normal takeoff/flying configuration

uint8 launch_detection_state

```
