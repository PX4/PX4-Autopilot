# SensorAirflow (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorAirflow.msg)

```c
uint64 timestamp		# time since system start (microseconds)
uint32 device_id                # unique device ID for the sensor that does not change between power cycles
float32 speed			# the speed being reported by the wind / airflow sensor
float32 direction		# the direction being reported by the wind / airflow sensor
uint8 status			# Status code from the sensor

```
