# RateCtrlStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RateCtrlStatus.msg)

```c
uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 wheel_rate_integ	# FW only and optional

```
