# ParameterSetValueRequest (UORB message)

ParameterSetValueRequest : Used by a remote or primary to update the value for a parameter at the other end

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetValueRequest.msg)

```c
# ParameterSetValueRequest : Used by a remote or primary to update the value for a parameter at the other end

uint64 timestamp
uint16 parameter_index

int32 int_value             # Optional value for an integer parameter
float32 float_value         # Optional value for a float parameter

uint8 ORB_QUEUE_LENGTH = 32

# TOPICS parameter_set_value_request parameter_remote_set_value_request parameter_primary_set_value_request

```
