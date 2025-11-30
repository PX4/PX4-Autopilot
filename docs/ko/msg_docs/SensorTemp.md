# SensorTemp (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorTemp.msg)

```c
uint64 timestamp          # time since system start (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
float32  temperature      # Temperature provided by sensor (Celsius)

```
