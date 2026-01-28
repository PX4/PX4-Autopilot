# EstimatorBias (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorBias.msg)

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
float32 bias			# estimated barometric altitude bias (m)
float32 bias_var		# estimated barometric altitude bias variance (m^2)

float32 innov			# innovation of the last measurement fusion (m)
float32 innov_var		# innovation variance of the last measurement fusion (m^2)
float32 innov_test_ratio	# normalized innovation squared test ratio

# TOPICS estimator_baro_bias estimator_gnss_hgt_bias

```
