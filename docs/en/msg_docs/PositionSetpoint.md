---
pageClass: is-wide-page
---

# PositionSetpoint (UORB message)

this file is only used in the position_setpoint triple as a dependency.

**TOPICS:** position_setpoint

## Fields

| Name                                                                                  | Type      | Unit [Frame] | Range/Enum | Description                                                                                       |
| ------------------------------------------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                   | `uint64`  |              |            | time since system start (microseconds)                                                            |
| <a id="fld_valid"></a>valid                                                           | `bool`    |              |            | true if setpoint is valid                                                                         |
| <a id="fld_type"></a>type                                                             | `uint8`   |              |            | setpoint type to adjust behavior of position controller                                           |
| <a id="fld_vx"></a>vx                                                                 | `float32` |              |            | local velocity setpoint in m/s in NED                                                             |
| <a id="fld_vy"></a>vy                                                                 | `float32` |              |            | local velocity setpoint in m/s in NED                                                             |
| <a id="fld_vz"></a>vz                                                                 | `float32` |              |            | local velocity setpoint in m/s in NED                                                             |
| <a id="fld_lat"></a>lat                                                               | `float64` |              |            | latitude, in deg                                                                                  |
| <a id="fld_lon"></a>lon                                                               | `float64` |              |            | longitude, in deg                                                                                 |
| <a id="fld_alt"></a>alt                                                               | `float32` |              |            | altitude AMSL, in m                                                                               |
| <a id="fld_yaw"></a>yaw                                                               | `float32` |              |            | yaw (only in hover), in rad [-PI..PI), NaN = leave to flight task                                 |
| <a id="fld_loiter_radius"></a>loiter_radius                                           | `float32` | m            | [0 : INF]  | loiter major axis radius                                                                          |
| <a id="fld_loiter_minor_radius"></a>loiter_minor_radius                               | `float32` | m            | [0 : INF]  | loiter minor axis radius (used for non-circular loiter shapes)                                    |
| <a id="fld_loiter_direction_counter_clockwise"></a>loiter_direction_counter_clockwise | `bool`    |              |            | loiter direction is clockwise by default and can be changed using this field                      |
| <a id="fld_loiter_orientation"></a>loiter_orientation                                 | `float32` | rad          | [-pi : pi] | orientation of the major axis with respect to true north                                          |
| <a id="fld_loiter_pattern"></a>loiter_pattern                                         | `uint8`   |              |            | loitern pattern to follow                                                                         |
| <a id="fld_acceptance_radius"></a>acceptance_radius                                   | `float32` |              |            | horizontal acceptance_radius (meters)                                                             |
| <a id="fld_alt_acceptance_radius"></a>alt_acceptance_radius                           | `float32` |              |            | vertical acceptance radius, only used for fixed wing guidance, NAN = let guidance choose (meters) |
| <a id="fld_course"></a>course                                                         | `float32` | rad          |            | desired course (bearing) over ground, NaN = unused                                                |
| <a id="fld_cruising_speed"></a>cruising_speed                                         | `float32` |              |            | the generally desired cruising speed (not a hard constraint)                                      |
| <a id="fld_gliding_enabled"></a>gliding_enabled                                       | `bool`    |              |            | commands the vehicle to glide if the capability is available (fixed wing only)                    |
| <a id="fld_cruising_throttle"></a>cruising_throttle                                   | `float32` |              |            | the generally desired cruising throttle (not a hard constraint), only has an effect for rover     |

## Constants

| Name                                                          | Type    | Value | Description                                                    |
| ------------------------------------------------------------- | ------- | ----- | -------------------------------------------------------------- |
| <a id="#SETPOINT_TYPE_POSITION"></a> SETPOINT_TYPE_POSITION   | `uint8` | 0     | position setpoint                                              |
| <a id="#SETPOINT_TYPE_VELOCITY"></a> SETPOINT_TYPE_VELOCITY   | `uint8` | 1     | velocity setpoint                                              |
| <a id="#SETPOINT_TYPE_LOITER"></a> SETPOINT_TYPE_LOITER       | `uint8` | 2     | loiter setpoint                                                |
| <a id="#SETPOINT_TYPE_TAKEOFF"></a> SETPOINT_TYPE_TAKEOFF     | `uint8` | 3     | takeoff setpoint                                               |
| <a id="#SETPOINT_TYPE_LAND"></a> SETPOINT_TYPE_LAND           | `uint8` | 4     | land setpoint, altitude must be ignored, descend until landing |
| <a id="#SETPOINT_TYPE_IDLE"></a> SETPOINT_TYPE_IDLE           | `uint8` | 5     | do nothing, switch off motors or keep at idle speed (MC)       |
| <a id="#LOITER_TYPE_ORBIT"></a> LOITER_TYPE_ORBIT             | `uint8` | 0     | Circular pattern                                               |
| <a id="#LOITER_TYPE_FIGUREEIGHT"></a> LOITER_TYPE_FIGUREEIGHT | `uint8` | 1     | Pattern resembling an 8                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionSetpoint.msg)

::: details Click here to see original file

```c
# this file is only used in the position_setpoint triple as a dependency

uint64 timestamp		# time since system start (microseconds)

uint8 SETPOINT_TYPE_POSITION=0	# position setpoint
uint8 SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
uint8 SETPOINT_TYPE_LOITER=2	# loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF=3	# takeoff setpoint
uint8 SETPOINT_TYPE_LAND=4	# land setpoint, altitude must be ignored, descend until landing
uint8 SETPOINT_TYPE_IDLE=5	# do nothing, switch off motors or keep at idle speed (MC)

uint8 LOITER_TYPE_ORBIT=0 	# Circular pattern
uint8 LOITER_TYPE_FIGUREEIGHT=1 # Pattern resembling an 8

bool valid			# true if setpoint is valid
uint8 type			# setpoint type to adjust behavior of position controller

float32 vx			# local velocity setpoint in m/s in NED
float32 vy			# local velocity setpoint in m/s in NED
float32 vz			# local velocity setpoint in m/s in NED

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only in hover), in rad [-PI..PI), NaN = leave to flight task

float32 loiter_radius		# [m] [@range 0, INF] loiter major axis radius
float32 loiter_minor_radius	# [m] [@range 0, INF] loiter minor axis radius (used for non-circular loiter shapes)
bool loiter_direction_counter_clockwise # loiter direction is clockwise by default and can be changed using this field
float32 loiter_orientation 	# [rad] [@range -pi, pi] orientation of the major axis with respect to true north
uint8 	loiter_pattern		# loitern pattern to follow

float32 acceptance_radius   # horizontal acceptance_radius (meters)
float32 alt_acceptance_radius # vertical acceptance radius, only used for fixed wing guidance, NAN = let guidance choose (meters)

float32 course			# [rad] desired course (bearing) over ground, NaN = unused
float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
bool gliding_enabled		# commands the vehicle to glide if the capability is available (fixed wing only)
float32 cruising_throttle	# the generally desired cruising throttle (not a hard constraint), only has an effect for rover
```

:::
