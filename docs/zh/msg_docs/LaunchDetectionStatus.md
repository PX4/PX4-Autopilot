---
pageClass: is-wide-page
---

# LaunchDetectionStatus (UORB message)

Status of the launch detection state machine (fixed-wing only).

**TOPICS:** launch_detectionstatus

## Fields

| 参数名                                                              | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ---------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                        | `uint64` |                                                                  |            | time since system start (microseconds) |
| launch_detection_state | `uint8`  |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                                                                                                           | 类型      | 值 | 描述                                                                                                                                                                      |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | - | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#STATE_WAITING_FOR_LAUNCH"></a> STATE_WAITING_FOR_LAUNCH                                              | `uint8` | 0 | waiting for launch                                                                                                                                                      |
| <a href="#STATE_LAUNCH_DETECTED_DISABLED_MOTOR"></a> STATE_LAUNCH_DETECTED_DISABLED_MOTOR | `uint8` | 1 | launch detected, but keep motor(s) disabled (e.g. because it can't spin freely while on catapult) |
| <a href="#STATE_FLYING"></a> STATE_FLYING                                                                                                                | `uint8` | 2 | launch detected, use normal takeoff/flying configuration                                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LaunchDetectionStatus.msg)

:::details
Click here to see original file

```c
# Status of the launch detection state machine (fixed-wing only)

uint64 timestamp # time since system start (microseconds)

uint8 STATE_WAITING_FOR_LAUNCH 			= 0 # waiting for launch
uint8 STATE_LAUNCH_DETECTED_DISABLED_MOTOR 	= 1 # launch detected, but keep motor(s) disabled (e.g. because it can't spin freely while on catapult)
uint8 STATE_FLYING 				= 2 # launch detected, use normal takeoff/flying configuration

uint8 launch_detection_state
```

:::
