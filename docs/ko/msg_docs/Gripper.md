---
pageClass: is-wide-page
---

# Gripper (UORB message)

# Used to command an actuation in the gripper, which is mapped to a specific output in the control allocation module.

**TOPICS:** gripper

## Fields

| 명칭        | 형식       | Unit [Frame] | Range/Enum | 설명                              |
| --------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------- |
| timestamp | `uint64` |                                                                  |            |                                 |
| command   | `int8`   |                                                                  |            | Commanded state for the gripper |

## Constants

| 명칭                                                                   | 형식     | Value | 설명 |
| -------------------------------------------------------------------- | ------ | ----- | -- |
| <a href="#COMMAND_GRAB"></a> COMMAND_GRAB       | `int8` | 0     |    |
| <a href="#COMMAND_RELEASE"></a> COMMAND_RELEASE | `int8` | 1     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Gripper.msg)

:::details
Click here to see original file

```c
## Used to command an actuation in the gripper, which is mapped to a specific output in the control allocation module

uint64 timestamp

int8 command		# Commanded state for the gripper
int8 COMMAND_GRAB = 0
int8 COMMAND_RELEASE = 1
```

:::
