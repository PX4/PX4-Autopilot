# Airspeed (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Airspeed.msg)

```c
uint64 timestamp                 # time since system start (microseconds)
uint64 timestamp_sample

float32 indicated_airspeed_m_s   # indicated airspeed in m/s

float32 true_airspeed_m_s        # true filtered airspeed in m/s

float32 confidence               # confidence value from 0 to 1 for this sensor

```
