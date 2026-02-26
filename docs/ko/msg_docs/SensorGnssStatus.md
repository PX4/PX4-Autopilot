---
pageClass: is-wide-page
---

# SensorGnssStatus (UORB message)

Gnss quality indicators.

**TOPICS:** sensor_gnssstatus

## Fields

| 명칭                                                                | 형식       | Unit [Frame] | Range/Enum                                      | 설명                                                                        |
| ----------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------- |
| timestamp                                                         | `uint64` |                                                                  |                                                 | time since system start (microseconds)                 |
| device_id                                    | `uint32` |                                                                  |                                                 | unique device ID for the sensor that does not change between power cycles |
| `bool`                                                            |          |                                                                  | Set to true if quality indicators are available |                                                                           |
| quality_corrections                          | `uint8`  |                                                                  |                                                 | Corrections quality from 0 to 10, or 255 if not available                 |
| quality_receiver                             | `uint8`  |                                                                  |                                                 | Overall receiver operating status from 0 to 10, or 255 if not available   |
| quality_gnss_signals    | `uint8`  |                                                                  |                                                 | Quality of GNSS signals from 0 to 10, or 255 if not available             |
| quality_post_processing | `uint8`  |                                                                  |                                                 | Expected post processing quality from 0 to 10, or 255 if not available    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGnssStatus.msg)

:::details
Click here to see original file

```c
# Gnss quality indicators

uint64 timestamp		# time since system start (microseconds)
uint32 device_id                # unique device ID for the sensor that does not change between power cycles

bool  quality_available         # Set to true if quality indicators are available
uint8 quality_corrections       # Corrections quality from 0 to 10, or 255 if not available
uint8 quality_receiver          # Overall receiver operating status from 0 to 10, or 255 if not available
uint8 quality_gnss_signals      # Quality of GNSS signals from 0 to 10, or 255 if not available
uint8 quality_post_processing   # Expected post processing quality from 0 to 10, or 255 if not available
```

:::
