# SensorSelection (UORB message)

Sensor ID's for the voted sensors output on the sensor_combined topic.
Will be updated on startup of the sensor module and when sensor selection changes

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorSelection.msg)

```c
#
# Sensor ID's for the voted sensors output on the sensor_combined topic.
# Will be updated on startup of the sensor module and when sensor selection changes
#
uint64 timestamp		# time since system start (microseconds)
uint32 accel_device_id		# unique device ID for the selected accelerometers
uint32 gyro_device_id		# unique device ID for the selected rate gyros

```
