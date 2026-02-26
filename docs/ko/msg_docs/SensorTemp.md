---
pageClass: is-wide-page
---

# SensorTemp (UORB message)

**TOPICS:** sensor_temp

## Fields

| 명칭                             | 형식       | Unit [Frame] | Range/Enum                                                  | 설명                                                                        |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |                                                             | time since system start (microseconds)                 |
| device_id | `uint32` |                                                                  |                                                             | unique device ID for the sensor that does not change between power cycles |
| `float32`                      |          |                                                                  | Temperature provided by sensor (Celsius) |                                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorTemp.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
float32  temperature      # Temperature provided by sensor (Celsius)
```

:::
