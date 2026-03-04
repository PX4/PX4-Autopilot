---
pageClass: is-wide-page
---

# MountOrientation (UORB message)

**TOPICS:** mount_orientation

## Fields

| 명칭                                                             | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
| -------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                      | `uint64`     |                                                                  |            | time since system start (microseconds) |
| attitude_euler_angle | `float32[3]` |                                                                  |            | Attitude/direction of the mount as euler angles in rad    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MountOrientation.msg)

:::details
Click here to see original file

```c
uint64 timestamp				# time since system start (microseconds)
float32[3] attitude_euler_angle		# Attitude/direction of the mount as euler angles in rad
```

:::
