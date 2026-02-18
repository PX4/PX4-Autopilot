---
pageClass: is-wide-page
---

# GpsInjectData (UORB message)

**TOPICS:** gps_injectdata

## Fields

| 명칭                             | 형식           | Unit [Frame] | Range/Enum | 설명                                                                        |
| ------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                      | `uint64`     |                                                                  |            | time since system start (microseconds)                 |
| device_id | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| len                            | `uint16`     |                                                                  |            | length of data                                                            |
| flags                          | `uint8`      |                                                                  |            | LSB: 1=fragmented                                         |
| data                           | `uint8[300]` |                                                                  |            | data to write to GPS device (RTCM message)             |

## Constants

| 명칭                                                                                          | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8     |    |
| <a href="#MAX_INSTANCES"></a> MAX_INSTANCES                            | `uint8` | 2     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpsInjectData.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

uint16 len                       # length of data
uint8 flags                     # LSB: 1=fragmented
uint8[300] data                 # data to write to GPS device (RTCM message)

uint8 ORB_QUEUE_LENGTH = 8

uint8 MAX_INSTANCES = 2
```

:::
