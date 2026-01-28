# GpioOut (повідомлення UORB)

Маска та стан GPIO

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioOut.msg)

```c
# GPIO mask and state

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 mask				# pin mask
uint32 state				# pin state mask

```
