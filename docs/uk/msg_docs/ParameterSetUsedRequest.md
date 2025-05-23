# ParameterSetUsedRequest (повідомлення UORB)

ParameterSetUsedRequest : Used by a remote to update the used flag for a parameter on the primary

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterSetUsedRequest.msg)

```c
# ParameterSetUsedRequest : Used by a remote to update the used flag for a parameter on the primary

uint64 timestamp
uint16 parameter_index

uint8 ORB_QUEUE_LENGTH = 64

```
