# Buffer128 (повідомлення UORB)

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Buffer128.msg)

```c
uint64 timestamp		# time since system start (microseconds)

uint8 len                       # length of data
uint32 MAX_BUFLEN = 128

uint8[128] data                 # data

# TOPICS voxl2_io_data

```
