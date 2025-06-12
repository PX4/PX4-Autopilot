# MagnetometerBiasEstimate (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MagnetometerBiasEstimate.msg)

```c
uint64 timestamp                # time since system start (microseconds)

float32[4] bias_x		# estimated X-bias of all the sensors
float32[4] bias_y		# estimated Y-bias of all the sensors
float32[4] bias_z		# estimated Z-bias of all the sensors

bool[4] valid			# true if the estimator has converged
bool[4] stable

```
