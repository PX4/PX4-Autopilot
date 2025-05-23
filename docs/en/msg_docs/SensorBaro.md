# SensorBaro (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorBaro.msg)

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 pressure          # static pressure measurement in Pascals

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 4

```
