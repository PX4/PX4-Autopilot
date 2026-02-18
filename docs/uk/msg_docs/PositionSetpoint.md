---
pageClass: is-wide-page
---

# PositionSetpoint (повідомлення UORB)

this file is only used in the position_setpoint triple as a dependency.

**TOPICS:** position_setpoint

## Fields

| Назва                                                                                             | Тип       | Unit [Frame] | Range/Enum                                                                     | Опис                                                                                                                                           |
| ------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                         | `uint64`  |                                                                  |                                                                                | time since system start (microseconds)                                                                                      |
| valid                                                                                             | `bool`    |                                                                  |                                                                                | true if setpoint is valid                                                                                                                      |
| type                                                                                              | `uint8`   |                                                                  |                                                                                | setpoint type to adjust behavior of position controller                                                                                        |
| vx                                                                                                | `float32` |                                                                  |                                                                                | local velocity setpoint in m/s in NED                                                                                                          |
| vy                                                                                                | `float32` |                                                                  |                                                                                | local velocity setpoint in m/s in NED                                                                                                          |
| vz                                                                                                | `float32` |                                                                  |                                                                                | local velocity setpoint in m/s in NED                                                                                                          |
| lat                                                                                               | `float64` |                                                                  |                                                                                | latitude, in deg                                                                                                                               |
| lon                                                                                               | `float64` |                                                                  |                                                                                | longitude, in deg                                                                                                                              |
| alt                                                                                               | `float32` |                                                                  |                                                                                | altitude AMSL, in m                                                                                                                            |
| yaw                                                                                               | `float32` |                                                                  |                                                                                | yaw (only in hover), in rad [-PI..PI), NaN = leave to flight task |
| loiter_radius                                                                | `float32` | m                                                                | [0 : INF]  | loiter major axis radius                                                                                                                       |
| loiter_minor_radius                                     | `float32` | m                                                                | [0 : INF]  | loiter minor axis radius (used for non-circular loiter shapes)                                                              |
| loiter_direction_counter_clockwise | `bool`    |                                                                  |                                                                                | loiter direction is clockwise by default and can be changed using this field                                                                   |
| loiter_orientation                                                           | `float32` | rad                                                              | [-pi : pi] | orientation of the major axis with respect to true north                                                                                       |
| loiter_pattern                                                               | `uint8`   |                                                                  |                                                                                | loitern pattern to follow                                                                                                                      |
| acceptance_radius                                                            | `float32` |                                                                  |                                                                                | horizontal acceptance_radius (meters)                                                                  |
| alt_acceptance_radius                                   | `float32` |                                                                  |                                                                                | vertical acceptance radius, only used for fixed wing guidance, NAN = let guidance choose (meters)                           |
| cruising_speed                                                               | `float32` |                                                                  |                                                                                | the generally desired cruising speed (not a hard constraint)                                                                |
| gliding_enabled                                                              | `bool`    |                                                                  |                                                                                | commands the vehicle to glide if the capability is available (fixed wing only)                                              |
| cruising_throttle                                                            | `float32` |                                                                  |                                                                                | the generally desired cruising throttle (not a hard constraint), only has an effect for rover                               |

## Constants

| Назва                                                                                                     | Тип     | Значення | Опис                                                                        |
| --------------------------------------------------------------------------------------------------------- | ------- | -------- | --------------------------------------------------------------------------- |
| <a href="#SETPOINT_TYPE_POSITION"></a> SETPOINT_TYPE_POSITION   | `uint8` | 0        | position setpoint                                                           |
| <a href="#SETPOINT_TYPE_VELOCITY"></a> SETPOINT_TYPE_VELOCITY   | `uint8` | 1        | velocity setpoint                                                           |
| <a href="#SETPOINT_TYPE_LOITER"></a> SETPOINT_TYPE_LOITER       | `uint8` | 2        | loiter setpoint                                                             |
| <a href="#SETPOINT_TYPE_TAKEOFF"></a> SETPOINT_TYPE_TAKEOFF     | `uint8` | 3        | takeoff setpoint                                                            |
| <a href="#SETPOINT_TYPE_LAND"></a> SETPOINT_TYPE_LAND           | `uint8` | 4        | land setpoint, altitude must be ignored, descend until landing              |
| <a href="#SETPOINT_TYPE_IDLE"></a> SETPOINT_TYPE_IDLE           | `uint8` | 5        | do nothing, switch off motors or keep at idle speed (MC) |
| <a href="#LOITER_TYPE_ORBIT"></a> LOITER_TYPE_ORBIT             | `uint8` | 0        | Circular pattern                                                            |
| <a href="#LOITER_TYPE_FIGUREEIGHT"></a> LOITER_TYPE_FIGUREEIGHT | `uint8` | 1        | Pattern resembling an 8                                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionSetpoint.msg)

:::details
Click here to see original file

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

float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
bool gliding_enabled		# commands the vehicle to glide if the capability is available (fixed wing only)
float32 cruising_throttle	# the generally desired cruising throttle (not a hard constraint), only has an effect for rover
```

:::
