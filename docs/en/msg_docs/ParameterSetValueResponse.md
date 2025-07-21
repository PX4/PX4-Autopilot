# ParameterSetValueResponse (UORB message)

ParameterSetValueResponse : Response to a set value request by either primary or secondary

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetValueResponse.msg)

```c
# ParameterSetValueResponse : Response to a set value request by either primary or secondary

uint64 timestamp
uint64 request_timestamp
uint16 parameter_index

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS parameter_set_value_response parameter_remote_set_value_response parameter_primary_set_value_response

```
