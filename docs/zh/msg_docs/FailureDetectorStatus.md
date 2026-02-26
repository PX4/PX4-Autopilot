---
pageClass: is-wide-page
---

# FailureDetectorStatus (UORB message)

**TOPICS:** failure_detectorstatus

## Fields

| 参数名                                                              | 类型        | Unit [Frame] | Range/Enum | 描述                                                                       |
| ---------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------ |
| timestamp                                                        | `uint64`  |                                                                  |            | time since system start (microseconds)                |
| fd_roll                                     | `bool`    |                                                                  |            |                                                                          |
| fd_pitch                                    | `bool`    |                                                                  |            |                                                                          |
| fd_alt                                      | `bool`    |                                                                  |            |                                                                          |
| fd_ext                                      | `bool`    |                                                                  |            |                                                                          |
| fd_arm_escs            | `bool`    |                                                                  |            |                                                                          |
| fd_battery                                  | `bool`    |                                                                  |            |                                                                          |
| fd_imbalanced_prop     | `bool`    |                                                                  |            |                                                                          |
| fd_motor                                    | `bool`    |                                                                  |            |                                                                          |
| imbalanced_prop_metric | `float32` |                                                                  |            | Metric of the imbalanced propeller check (low-passed) |
| motor_failure_mask     | `uint16`  |                                                                  |            | Bit-mask with motor indices, indicating critical motor failures          |
| motor_stop_mask        | `uint16`  |                                                                  |            | Bitmaks of motors stopped by failure injection                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FailureDetectorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp                    # time since system start (microseconds)

# FailureDetector status
bool fd_roll
bool fd_pitch
bool fd_alt
bool fd_ext
bool fd_arm_escs
bool fd_battery
bool fd_imbalanced_prop
bool fd_motor

float32 imbalanced_prop_metric      # Metric of the imbalanced propeller check (low-passed)
uint16 motor_failure_mask           # Bit-mask with motor indices, indicating critical motor failures
uint16 motor_stop_mask              # Bitmaks of motors stopped by failure injection
```

:::
