---
pageClass: is-wide-page
---

# ManualControlSetpoint (UORB message)

**TOPICS:** manual_control_setpoint manual_control_input

## Fields

| 명칭                                                                     | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                |
| ---------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                    | `uint64`  |                                                                  |            | time since system start (microseconds)                                         |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  |                                                                  |            | the timestamp of the raw data (microseconds)                                   |
| <a id="fld_valid"></a>valid                                            | `bool`    |                                                                  |            |                                                                                                   |
| <a id="fld_data_source"></a>data_source           | `uint8`   |                                                                  |            |                                                                                                   |
| <a id="fld_roll"></a>roll                                              | `float32` |                                                                  |            | move right, positive roll rotation, right side down                                               |
| <a id="fld_pitch"></a>pitch                                            | `float32` |                                                                  |            | move forward, negative pitch rotation, nose down                                                  |
| <a id="fld_yaw"></a>yaw                                                | `float32` |                                                                  |            | positive yaw rotation, clockwise when seen top down                                               |
| <a id="fld_throttle"></a>throttle                                      | `float32` |                                                                  |            | move up, positive thrust, -1 is minimum available 0% or -100% +1 is 100% thrust                   |
| <a id="fld_flaps"></a>flaps                                            | `float32` |                                                                  |            | position of flaps switch/knob/lever [-1, 1]   |
| <a id="fld_aux1"></a>aux1                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_aux2"></a>aux2                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_aux3"></a>aux3                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_aux4"></a>aux4                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_aux5"></a>aux5                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_aux6"></a>aux6                                              | `float32` |                                                                  |            |                                                                                                   |
| <a id="fld_sticks_moving"></a>sticks_moving       | `bool`    |                                                                  |            | manual control override request in an auto or offboard mode, only gets true if feature is enabled |
| <a id="fld_buttons"></a>buttons                                        | `uint16`  |                                                                  |            | From uint16 buttons field of Mavlink manual_control message                  |

## Constants

| 명칭                                                                                        | 형식       | Value | 설명                                                               |
| ----------------------------------------------------------------------------------------- | -------- | ----- | ---------------------------------------------------------------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION                        | `uint32` | 0     |                                                                  |
| <a id="#SOURCE_UNKNOWN"></a> SOURCE_UNKNOWN                          | `uint8`  | 0     |                                                                  |
| <a id="#SOURCE_RC"></a> SOURCE_RC                                    | `uint8`  | 1     | radio control (input_rc) |
| <a id="#SOURCE_MAVLINK_0"></a> SOURCE_MAVLINK_0 | `uint8`  | 2     | mavlink instance 0                                               |
| <a id="#SOURCE_MAVLINK_1"></a> SOURCE_MAVLINK_1 | `uint8`  | 3     | mavlink instance 1                                               |
| <a id="#SOURCE_MAVLINK_2"></a> SOURCE_MAVLINK_2 | `uint8`  | 4     | mavlink instance 2                                               |
| <a id="#SOURCE_MAVLINK_3"></a> SOURCE_MAVLINK_3 | `uint8`  | 5     | mavlink instance 3                                               |
| <a id="#SOURCE_MAVLINK_4"></a> SOURCE_MAVLINK_4 | `uint8`  | 6     | mavlink instance 4                                               |
| <a id="#SOURCE_MAVLINK_5"></a> SOURCE_MAVLINK_5 | `uint8`  | 7     | mavlink instance 5                                               |

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

bool sticks_moving # manual control override request in an auto or offboard mode, only gets true if feature is enabled

uint16 buttons		# From uint16 buttons field of Mavlink manual_control message

# TOPICS manual_control_setpoint manual_control_input
# DEPRECATED: float32 x
# DEPRECATED: float32 y
# DEPRECATED: float32 z
# DEPRECATED: float32 r
```

:::
