# QshellReq (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/QshellReq.msg)

```c
uint64 timestamp		# time since system start (microseconds)
char[100] cmd
uint32 MAX_STRLEN = 100
uint32 strlen
uint32 request_sequence

```
