---
pageClass: is-wide-page
---

# GotoSetpoint (повідомлення UORB)

Position and (optional) heading setpoints with corresponding speed constraints. Setpoints are intended as inputs to position and heading smoothers, respectively. Setpoints do not need to be kinematically consistent. Optional heading setpoints may be specified as controlled by the respective flag. Unset optional setpoints are not controlled. Unset optional constraints default to vehicle specifications.

**TOPICS:** goto_setpoint

## Fields

| Назва                                                                                                             | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                         | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                       |
| положення                                                                                                         | `float32[3]` | m                                                                |            | NED local world frame                                                                                                                                           |
| flag_control_heading                                                    | `bool`       |                                                                  |            | true if heading is to be controlled                                                                                                                             |
| heading                                                                                                           | `float32`    |                                                                  |            | (optional) [rad] [-pi,pi] from North |
| flag_set_max_horizontal_speed | `bool`       |                                                                  |            | true if setting a non-default horizontal speed limit                                                                                                            |
| max_horizontal_speed                                                    | `float32`    |                                                                  |            | (optional) [m/s] maximum speed (absolute) in the NE-plane             |
| flag_set_max_vertical_speed   | `bool`       |                                                                  |            | true if setting a non-default vertical speed limit                                                                                                              |
| max_vertical_speed                                                      | `float32`    |                                                                  |            | (optional) [m/s] maximum speed (absolute) in the D-axis               |
| flag_set_max_heading_rate     | `bool`       |                                                                  |            | true if setting a non-default heading rate limit                                                                                                                |
| max_heading_rate                                                        | `float32`    |                                                                  |            | (optional) [rad/s] maximum heading rate (absolute)                    |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/GotoSetpoint.msg)

:::details
Click here to see original file

```c
# Position and (optional) heading setpoints with corresponding speed constraints
# Setpoints are intended as inputs to position and heading smoothers, respectively
# Setpoints do not need to be kinematically consistent
# Optional heading setpoints may be specified as controlled by the respective flag
# Unset optional setpoints are not controlled
# Unset optional constraints default to vehicle specifications

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# setpoints
float32[3] position # [m] NED local world frame

bool flag_control_heading # true if heading is to be controlled
float32 heading # (optional) [rad] [-pi,pi] from North

# constraints
bool flag_set_max_horizontal_speed # true if setting a non-default horizontal speed limit
float32 max_horizontal_speed # (optional) [m/s] maximum speed (absolute) in the NE-plane

bool flag_set_max_vertical_speed # true if setting a non-default vertical speed limit
float32 max_vertical_speed # (optional) [m/s] maximum speed (absolute) in the D-axis

bool flag_set_max_heading_rate # true if setting a non-default heading rate limit
float32 max_heading_rate # (optional) [rad/s] maximum heading rate (absolute)
```

:::
