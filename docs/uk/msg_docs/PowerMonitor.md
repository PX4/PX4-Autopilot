# PowerMonitor (повідомлення UORB)

power monitor message

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PowerMonitor.msg)

```c
# power monitor message

uint64 timestamp			# Time since system start (microseconds)

float32 voltage_v			# Voltage in volts, 0 if unknown
float32 current_a		    # Current in amperes, -1 if unknown
float32 power_w		        # power in watts, -1 if unknown
int16 rconf
int16 rsv
int16 rbv
int16 rp
int16 rc
int16 rcal
int16 me
int16 al

```
