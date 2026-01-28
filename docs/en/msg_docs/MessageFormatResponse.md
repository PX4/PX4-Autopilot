# MessageFormatResponse (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MessageFormatResponse.msg)

```c
uint64 timestamp # time since system start (microseconds)

# Response from PX4 with the format of a message

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command

bool success
uint32 message_hash # hash over all message fields

```
