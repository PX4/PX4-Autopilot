# SensorMag (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorMag.msg)

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # magnetic field in the FRD board frame X-axis in Gauss
float32 y                 # magnetic field in the FRD board frame Y-axis in Gauss
float32 z                 # magnetic field in the FRD board frame Z-axis in Gauss

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 4

```
