---
pageClass: is-wide-page
---

# SensorHygrometer (повідомлення UORB)

**TOPICS:** sensor_hygrometer

## Fields

| Назва                                 | Тип       | Unit [Frame] | Range/Enum                                                  | Опис                                                                      |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`  |                                                                  |                                                             | time since system start (microseconds)                 |
| timestamp_sample | `uint64`  |                                                                  |                                                             |                                                                           |
| device_id        | `uint32`  |                                                                  |                                                             | unique device ID for the sensor that does not change between power cycles |
| `float32`                             |           |                                                                  | Temperature provided by sensor (Celsius) |                                                                           |
| humidity                              | `float32` |                                                                  |                                                             | Humidity provided by sensor                                               |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorHygrometer.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32  temperature      # Temperature provided by sensor (Celsius)

float32 humidity          # Humidity provided by sensor
```

:::
