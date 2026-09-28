---
pageClass: is-wide-page
---

# HomePositionV1 (UORB message)

GPS home position in WGS84 coordinates.

**TOPICS:** home_position_v1

## Fields

| Name                                      | Type      | Unit [Frame] | Range/Enum | Description                                        |
| ----------------------------------------- | --------- | ------------ | ---------- | -------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp       | `uint64`  |              |            | time since system start (microseconds)             |
| <a id="fld_lat"></a>lat                   | `float64` |              |            | Latitude in degrees                                |
| <a id="fld_lon"></a>lon                   | `float64` |              |            | Longitude in degrees                               |
| <a id="fld_alt"></a>alt                   | `float32` |              |            | Altitude in meters (AMSL)                          |
| <a id="fld_x"></a>x                       | `float32` |              |            | X coordinate in meters                             |
| <a id="fld_y"></a>y                       | `float32` |              |            | Y coordinate in meters                             |
| <a id="fld_z"></a>z                       | `float32` |              |            | Z coordinate in meters                             |
| <a id="fld_roll"></a>roll                 | `float32` |              |            | Pitch angle in radians                             |
| <a id="fld_pitch"></a>pitch               | `float32` |              |            | Roll angle in radians                              |
| <a id="fld_yaw"></a>yaw                   | `float32` |              |            | Yaw angle in radians                               |
| <a id="fld_valid_alt"></a>valid_alt       | `bool`    |              |            | true when the altitude has been set                |
| <a id="fld_valid_hpos"></a>valid_hpos     | `bool`    |              |            | true when the latitude and longitude have been set |
| <a id="fld_valid_lpos"></a>valid_lpos     | `bool`    |              |            | true when the local position (xyz) has been set    |
| <a id="fld_manual_home"></a>manual_home   | `bool`    |              |            | true when home position was set manually           |
| <a id="fld_update_count"></a>update_count | `uint32`  |              |            | update counter of the home position                |

## Constants

| Name                                          | Type     | Value | Description |
| --------------------------------------------- | -------- | ----- | ----------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 1     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/HomePositionV1.msg)

::: details Click here to see original file

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
