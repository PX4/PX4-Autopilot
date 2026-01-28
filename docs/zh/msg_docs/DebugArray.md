# DebugArray (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugArray.msg)

```c
uint8 ARRAY_SIZE = 58
uint64 timestamp            # time since system start (microseconds)
uint16 id                   # unique ID of debug array, used to discriminate between arrays
char[10] name               # name of the debug array (max. 10 characters)
float32[58] data            # data

```
