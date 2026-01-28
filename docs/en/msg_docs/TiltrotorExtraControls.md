# TiltrotorExtraControls (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TiltrotorExtraControls.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 collective_tilt_normalized_setpoint	# Collective tilt angle of motors of tiltrotor, 0: vertical, 1: horizontal [0, 1]
float32 collective_thrust_normalized_setpoint 	# Collective thrust setpoint [0, 1]

```
