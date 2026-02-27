---
pageClass: is-wide-page
---

# VehicleRoi (UORB message)

Vehicle Region Of Interest (ROI).

**TOPICS:** vehicle_roi

## Fields

| 명칭                                | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| --------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64`  |                                                                  |            | time since system start (microseconds) |
| mode                              | `uint8`   |                                                                  |            | ROI mode (see above)                   |
| lat                               | `float64` |                                                                  |            | Latitude to point to                                      |
| lon                               | `float64` |                                                                  |            | Longitude to point to                                     |
| alt                               | `float32` |                                                                  |            | Altitude to point to                                      |
| roll_offset  | `float32` |                                                                  |            | angle offset in rad                                       |
| pitch_offset | `float32` |                                                                  |            | angle offset in rad                                       |
| yaw_offset   | `float32` |                                                                  |            | angle offset in rad                                       |

## Constants

| 명칭                                                                                  | 형식      | Value | 설명                                             |
| ----------------------------------------------------------------------------------- | ------- | ----- | ---------------------------------------------- |
| <a href="#ROI_NONE"></a> ROI_NONE                              | `uint8` | 0     | No region of interest                          |
| <a href="#ROI_WPNEXT"></a> ROI_WPNEXT                          | `uint8` | 1     | Point toward next MISSION with optional offset |
| <a href="#ROI_WPINDEX"></a> ROI_WPINDEX                        | `uint8` | 2     | Point toward given MISSION                     |
| <a href="#ROI_LOCATION"></a> ROI_LOCATION                      | `uint8` | 3     | Point toward fixed location                    |
| <a href="#ROI_TARGET"></a> ROI_TARGET                          | `uint8` | 4     | Point toward target                            |
| <a href="#ROI_ENUM_END"></a> ROI_ENUM_END | `uint8` | 5     |                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleRoi.msg)

:::details
Click here to see original file

```c
# Vehicle Region Of Interest (ROI)

uint64 timestamp			# time since system start (microseconds)

uint8 ROI_NONE = 0			# No region of interest
uint8 ROI_WPNEXT = 1			# Point toward next MISSION with optional offset
uint8 ROI_WPINDEX = 2			# Point toward given MISSION
uint8 ROI_LOCATION = 3			# Point toward fixed location
uint8 ROI_TARGET = 4			# Point toward target
uint8 ROI_ENUM_END = 5

uint8 mode          # ROI mode (see above)

float64 lat			    # Latitude to point to
float64 lon			    # Longitude to point to
float32 alt			    # Altitude to point to

# additional angle offsets to next waypoint (only used with ROI_WPNEXT)
float32 roll_offset		# angle offset in rad
float32 pitch_offset		# angle offset in rad
float32 yaw_offset		# angle offset in rad
```

:::
