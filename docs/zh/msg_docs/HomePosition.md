---
pageClass: is-wide-page
---

# HomePosition (UORB message)

GPS home position in WGS84 coordinates.

**TOPICS:** home_position

## Fields

| 参数名                               | 类型        | Unit [Frame] | Range/Enum | 描述                                                                 |
| --------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------ |
| timestamp                         | `uint64`  |                                                                  |            | time since system start (microseconds)          |
| lat                               | `float64` |                                                                  |            | Latitude in degrees                                                |
| lon                               | `float64` |                                                                  |            | Longitude in degrees                                               |
| alt                               | `float32` |                                                                  |            | Altitude in meters (AMSL)                       |
| x                                 | `float32` |                                                                  |            | X coordinate in meters                                             |
| y                                 | `float32` |                                                                  |            | Y coordinate in meters                                             |
| z                                 | `float32` |                                                                  |            | Z coordinate in meters                                             |
| roll                              | `float32` |                                                                  |            | Pitch angle in radians                                             |
| pitch                             | `float32` |                                                                  |            | Roll angle in radians                                              |
| yaw                               | `float32` |                                                                  |            | Yaw angle in radians                                               |
| valid_alt    | `bool`    |                                                                  |            | true when the altitude has been set                                |
| valid_hpos   | `bool`    |                                                                  |            | true when the latitude and longitude have been set                 |
| valid_lpos   | `bool`    |                                                                  |            | true when the local position (xyz) has been set |
| manual_home  | `bool`    |                                                                  |            | true when home position was set manually                           |
| update_count | `uint32`  |                                                                  |            | update counter of the home position                                |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 1 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/HomePosition.msg)

:::details
Click here to see original file

```c
# GPS home position in WGS84 coordinates.

uint32 MESSAGE_VERSION = 1

uint64 timestamp			# time since system start (microseconds)

float64 lat				# Latitude in degrees
float64 lon				# Longitude in degrees
float32 alt				# Altitude in meters (AMSL)

float32 x				# X coordinate in meters
float32 y				# Y coordinate in meters
float32 z				# Z coordinate in meters

float32 roll				# Pitch angle in radians
float32 pitch				# Roll angle in radians
float32 yaw				# Yaw angle in radians

bool valid_alt		# true when the altitude has been set
bool valid_hpos		# true when the latitude and longitude have been set
bool valid_lpos		# true when the local position (xyz) has been set

bool manual_home	# true when home position was set manually

uint32 update_count 	# update counter of the home position
```

:::
