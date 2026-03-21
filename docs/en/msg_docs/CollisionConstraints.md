---
pageClass: is-wide-page
---

# CollisionConstraints (UORB message)

Local setpoint constraints in NED frame. setting something to NaN means that no limit is provided.

**TOPICS:** collision_constraints

## Fields

| Name              | Type         | Unit [Frame] | Range/Enum | Description                            |
| ----------------- | ------------ | ------------ | ---------- | -------------------------------------- |
| timestamp         | `uint64`     |              |            | time since system start (microseconds) |
| original_setpoint | `float32[2]` |              |            | velocities demanded                    |
| adapted_setpoint  | `float32[2]` |              |            | velocities allowed                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CollisionConstraints.msg)

::: details Click here to see original file

```c
# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp	# time since system start (microseconds)

float32[2] original_setpoint   # velocities demanded
float32[2] adapted_setpoint    # velocities allowed
```

:::
