---
pageClass: is-wide-page
---

# QshellReq (UORB message)

**TOPICS:** qshell_req

## Fields

| 명칭                                    | 형식          | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                             | `uint64`    |                                                                  |            | time since system start (microseconds) |
| cmd                                   | `char[100]` |                                                                  |            |                                                           |
| strlen                                | `uint32`    |                                                                  |            |                                                           |
| request_sequence | `uint32`    |                                                                  |            |                                                           |

## Constants

| 명칭                                                         | 형식       | Value | 설명 |
| ---------------------------------------------------------- | -------- | ----- | -- |
| <a href="#MAX_STRLEN"></a> MAX_STRLEN | `uint32` | 100   |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/QshellReq.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
char[100] cmd
uint32 MAX_STRLEN = 100
uint32 strlen
uint32 request_sequence
```

:::
