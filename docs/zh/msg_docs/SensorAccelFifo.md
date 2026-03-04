---
pageClass: is-wide-page
---

# SensorAccelFifo (UORB message)

**TOPICS:** sensor_accelfifo

## Fields

| 参数名                                   | 类型          | Unit [Frame] | Range/Enum | 描述                                                                        |
| ------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`    |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample | `uint64`    |                                                                  |            |                                                                           |
| device_id        | `uint32`    |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| dt                                    | `float32`   |                                                                  |            | delta time between samples (microseconds)              |
| scale                                 | `float32`   |                                                                  |            |                                                                           |
| samples                               | `uint8`     |                                                                  |            | number of valid samples                                                   |
| x                                     | `int16[32]` |                                                                  |            | acceleration in the FRD board frame X-axis in m/s^2                       |
| y                                     | `int16[32]` |                                                                  |            | acceleration in the FRD board frame Y-axis in m/s^2                       |
| z                                     | `int16[32]` |                                                                  |            | acceleration in the FRD board frame Z-axis in m/s^2                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorAccelFifo.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[32] x               # acceleration in the FRD board frame X-axis in m/s^2
int16[32] y               # acceleration in the FRD board frame Y-axis in m/s^2
int16[32] z               # acceleration in the FRD board frame Z-axis in m/s^2
```

:::
