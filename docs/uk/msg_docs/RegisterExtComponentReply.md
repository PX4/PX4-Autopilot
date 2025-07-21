# RegisterExtComponentReply (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/RegisterExtComponentReply.msg)

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint64 request_id          # ID from the request
char[25] name              # name from the request

uint16 px4_ros2_api_version

bool success
int8 arming_check_id      # arming check registration ID (-1 if invalid)
int8 mode_id              # assigned mode ID (-1 if invalid)
int8 mode_executor_id     # assigned mode executor ID (-1 if invalid)

uint8 ORB_QUEUE_LENGTH = 2

```
