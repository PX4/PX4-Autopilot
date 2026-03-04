---
pageClass: is-wide-page
---

# SensorGyroFifo (UORB message)

**TOPICS:** sensor_gyrofifo

## Fields

| 参数名                                   | 类型          | Unit [Frame] | Range/Enum | 描述                                                                        |
| ------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`    |                                                                  |            | time since system start (microseconds)                 |
| timestamp_sample | `uint64`    |                                                                  |            |                                                                           |
| device_id        | `uint32`    |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| dt                                    | `float32`   |                                                                  |            | delta time between samples (microseconds)              |
| scale                                 | `float32`   |                                                                  |            |                                                                           |
| samples                               | `uint8`     |                                                                  |            | number of valid samples                                                   |
| x                                     | `int16[32]` |                                                                  |            | angular velocity in the FRD board frame X-axis in rad/s                   |
| y                                     | `int16[32]` |                                                                  |            | angular velocity in the FRD board frame Y-axis in rad/s                   |
| z                                     | `int16[32]` |                                                                  |            | angular velocity in the FRD board frame Z-axis in rad/s                   |

## Constants

| 参数名                                                                                         | 类型      | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGyroFifo.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[32] x               # angular velocity in the FRD board frame X-axis in rad/s
int16[32] y               # angular velocity in the FRD board frame Y-axis in rad/s
int16[32] z               # angular velocity in the FRD board frame Z-axis in rad/s

uint8 ORB_QUEUE_LENGTH = 4
```

:::
