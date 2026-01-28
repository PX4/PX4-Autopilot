# NavigatorStatus (UORB message)

Current status of a Navigator mode
The possible values of nav_state are defined in the VehicleStatus msg.

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NavigatorStatus.msg)

```c
# Current status of a Navigator mode
# The possible values of nav_state are defined in the VehicleStatus msg.
uint64 timestamp  # time since system start (microseconds)

uint8 nav_state   # Source mode (values in VehicleStatus)
uint8 failure     # Navigator failure enum

uint8 FAILURE_NONE = 0
uint8 FAILURE_HAGL = 1 # Target altitude exceeds maximum height above ground

```
