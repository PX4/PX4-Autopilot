# UnregisterExtComponent (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/UnregisterExtComponent.msg)

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

char[25] name                      # either the mode name, or component name

int8 arming_check_id      # arming check registration ID (-1 if not registered)
int8 mode_id              # assigned mode ID (-1 if not registered)
int8 mode_executor_id     # assigned mode executor ID (-1 if not registered)

```
