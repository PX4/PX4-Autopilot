---
pageClass: is-wide-page
---

# LateralControlConfiguration (UORB message)

Fixed Wing Lateral Control Configuration message. Used by the fw_lateral_longitudinal_control module to constrain FixedWingLateralSetpoint messages.

**TOPICS:** lateral_controlconfiguration

## Fields

| Назва                                                       | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                                      |
| ----------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                   | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                 |
| lateral_accel_max | `float32` | m/s^2                                                            |            | currently maps to a maximum roll angle, accel_max = tan(roll_max) \* GRAVITY |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/LateralControlConfiguration.msg)

:::details
Click here to see original file

```c
# Fixed Wing Lateral Control Configuration message
# Used by the fw_lateral_longitudinal_control module to constrain FixedWingLateralSetpoint messages.

uint32 MESSAGE_VERSION = 0

uint64 timestamp          # time since system start (microseconds)

float32 lateral_accel_max # [m/s^2] currently maps to a maximum roll angle, accel_max = tan(roll_max) * GRAVITY
```

:::
