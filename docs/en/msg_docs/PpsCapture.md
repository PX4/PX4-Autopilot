# PpsCapture (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PpsCapture.msg)

```c
uint64 timestamp			  # time since system start (microseconds) at PPS capture event
uint64 rtc_timestamp		# Corrected GPS UTC timestamp at PPS capture event
uint8  pps_rate_exceeded_counter # Increments when PPS dt < 50ms

```
