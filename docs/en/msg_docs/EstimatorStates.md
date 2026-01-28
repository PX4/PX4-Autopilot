# EstimatorStates (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStates.msg)

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[25] states		# Internal filter states
uint8 n_states		# Number of states effectively used

float32[24] covariances	# Diagonal Elements of Covariance Matrix

```
