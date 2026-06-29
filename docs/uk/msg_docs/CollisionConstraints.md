---
pageClass: is-wide-page
---

# CollisionConstraints (повідомлення UORB)

Local setpoint constraints in NED frame. setting something to NaN means that no limit is provided.

**TOPICS:** collision_constraints

## Fields

| Назва                                                                    | Тип          | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                      | `uint64`     |                                                                  |            | time since system start (microseconds) |
| <a id="fld_original_setpoint"></a>original_setpoint | `float32[2]` |                                                                  |            | velocities demanded                                       |
| <a id="fld_adapted_setpoint"></a>adapted_setpoint   | `float32[2]` |                                                                  |            | velocities allowed                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CollisionConstraints.msg)

:::details
Click here to see original file

```c
# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp	# time since system start (microseconds)

float32[2] original_setpoint   # velocities demanded
float32[2] adapted_setpoint    # velocities allowed
```

:::
