---
pageClass: is-wide-page
---

# ManualControlSetpoint (Повідомлення UORB)

**TOPICS:** manual_control_setpoint manual_control_input

## Fields

| Назва                                 | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                            |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`  |                                                                  |            | time since system start (microseconds)                                       |
| timestamp_sample | `uint64`  |                                                                  |            | the timestamp of the raw data (microseconds)                                 |
| valid                                 | `bool`    |                                                                  |            |                                                                                                 |
| data_source      | `uint8`   |                                                                  |            |                                                                                                 |
| roll                                  | `float32` |                                                                  |            | move right, positive roll rotation, right side down                                             |
| pitch                                 | `float32` |                                                                  |            | move forward, negative pitch rotation, nose down                                                |
| yaw                                   | `float32` |                                                                  |            | positive yaw rotation, clockwise when seen top down                                             |
| throttle                              | `float32` |                                                                  |            | move up, positive thrust, -1 is minimum available 0% or -100% +1 is 100% thrust                 |
| flaps                                 | `float32` |                                                                  |            | position of flaps switch/knob/lever [-1, 1] |
| aux1                                  | `float32` |                                                                  |            |                                                                                                 |
| aux2                                  | `float32` |                                                                  |            |                                                                                                 |
| aux3                                  | `float32` |                                                                  |            |                                                                                                 |
| aux4                                  | `float32` |                                                                  |            |                                                                                                 |
| aux5                                  | `float32` |                                                                  |            |                                                                                                 |
| aux6                                  | `float32` |                                                                  |            |                                                                                                 |
| sticks_moving    | `bool`    |                                                                  |            |                                                                                                 |
| buttons                               | `uint16`  |                                                                  |            | From uint16 buttons field of Mavlink manual_control message                |

## Constants

| Назва                                                                                       | Тип      | Значення | Опис                                                             |
| ------------------------------------------------------------------------------------------- | -------- | -------- | ---------------------------------------------------------------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                        | `uint32` | 0        |                                                                  |
| <a href="#SOURCE_UNKNOWN"></a> SOURCE_UNKNOWN                          | `uint8`  | 0        |                                                                  |
| <a href="#SOURCE_RC"></a> SOURCE_RC                                    | `uint8`  | 1        | radio control (input_rc) |
| <a href="#SOURCE_MAVLINK_0"></a> SOURCE_MAVLINK_0 | `uint8`  | 2        | mavlink instance 0                                               |
| <a href="#SOURCE_MAVLINK_1"></a> SOURCE_MAVLINK_1 | `uint8`  | 3        | mavlink instance 1                                               |
| <a href="#SOURCE_MAVLINK_2"></a> SOURCE_MAVLINK_2 | `uint8`  | 4        | mavlink instance 2                                               |
| <a href="#SOURCE_MAVLINK_3"></a> SOURCE_MAVLINK_3 | `uint8`  | 5        | mavlink instance 3                                               |
| <a href="#SOURCE_MAVLINK_4"></a> SOURCE_MAVLINK_4 | `uint8`  | 6        | mavlink instance 4                                               |
| <a href="#SOURCE_MAVLINK_5"></a> SOURCE_MAVLINK_5 | `uint8`  | 7        | mavlink instance 5                                               |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ManualControlSetpoint.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

bool valid

uint8 SOURCE_UNKNOWN   = 0
uint8 SOURCE_RC        = 1		# radio control (input_rc)
uint8 SOURCE_MAVLINK_0 = 2		# mavlink instance 0
uint8 SOURCE_MAVLINK_1 = 3		# mavlink instance 1
uint8 SOURCE_MAVLINK_2 = 4		# mavlink instance 2
uint8 SOURCE_MAVLINK_3 = 5		# mavlink instance 3
uint8 SOURCE_MAVLINK_4 = 6		# mavlink instance 4
uint8 SOURCE_MAVLINK_5 = 7		# mavlink instance 5

uint8 data_source

# Any of the channels may not be available and be set to NaN
# to indicate that it does not contain valid data.

# Stick positions [-1,1]
# on a common RC mode 1/2/3/4 remote/joystick the stick deflection: -1 is down/left, 1 is up/right
# Note: QGC sends throttle/z in range [0,1000] - [0,1]. The MAVLink input conversion [0,1] to [-1,1] is at the moment kept backwards compatible.
# Positive values are generally used for:
float32 roll     # move right,   positive roll rotation,  right side down
float32 pitch    # move forward, negative pitch rotation, nose down
float32 yaw      #               positive yaw rotation,   clockwise when seen top down
float32 throttle # move up,      positive thrust,         -1 is minimum available 0% or -100% +1 is 100% thrust

float32 flaps			 # position of flaps switch/knob/lever [-1, 1]

float32 aux1
float32 aux2
float32 aux3
float32 aux4
float32 aux5
float32 aux6

bool sticks_moving

uint16 buttons		# From uint16 buttons field of Mavlink manual_control message

# TOPICS manual_control_setpoint manual_control_input
# DEPRECATED: float32 x
# DEPRECATED: float32 y
# DEPRECATED: float32 z
# DEPRECATED: float32 r
```

:::
