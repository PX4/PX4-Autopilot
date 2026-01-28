# UavcanParameterValue (UORB message)

UAVCAN-MAVLink parameter bridge response type

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UavcanParameterValue.msg)

```c
# UAVCAN-MAVLink parameter bridge response type
uint64 timestamp		# time since system start (microseconds)
uint8 node_id			# UAVCAN node ID mapped from MAVLink component ID
char[17] param_id		# MAVLink/UAVCAN parameter name
int16 param_index		# parameter index, if known
uint16 param_count		# number of parameters exposed by the node
uint8 param_type		# MAVLink parameter type
int64 int_value			# current value if param_type is int-like
float32 real_value		# current value if param_type is float-like

```
