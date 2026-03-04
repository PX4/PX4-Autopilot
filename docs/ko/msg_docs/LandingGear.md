---
pageClass: is-wide-page
---

# LandingGear (UORB message)

**TOPICS:** landing_gear

## Fields

| 명칭                                | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| --------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64` |                                                                  |            | time since system start (microseconds) |
| landing_gear | `int8`   |                                                                  |            |                                                           |

## Constants

| 명칭                                                       | 형식     | Value | 설명                     |
| -------------------------------------------------------- | ------ | ----- | ---------------------- |
| <a href="#GEAR_UP"></a> GEAR_UP     | `int8` | 1     | landing gear up        |
| <a href="#GEAR_DOWN"></a> GEAR_DOWN | `int8` | -1    | landing gear down      |
| <a href="#GEAR_KEEP"></a> GEAR_KEEP | `int8` | 0     | keep the current state |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingGear.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

int8 GEAR_UP = 1 # landing gear up
int8 GEAR_DOWN = -1 # landing gear down
int8 GEAR_KEEP = 0 # keep the current state

int8 landing_gear
```

:::
