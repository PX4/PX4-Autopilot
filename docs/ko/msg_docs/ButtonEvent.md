# ButtonEvent (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ButtonEvent.msg)

```c
uint64 timestamp			# time since system start (microseconds)
bool triggered				# Set to true if the event is triggered

# TOPICS button_event safety_button

uint8 ORB_QUEUE_LENGTH = 2

```
