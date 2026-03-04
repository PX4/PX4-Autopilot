---
pageClass: is-wide-page
---

# ConfigOverridesV0 (UORB message)

Configurable overrides by (external) modes or mode executors.

**TOPICS:** config_overrides config_overrides_request

## Fields

| 参数名                                                                                      | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                              |
| ---------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------- |
| timestamp                                                                                | `uint64` |                                                                  |            | time since system start (microseconds)                                       |
| disable_auto_disarm                            | `bool`   |                                                                  |            | Prevent the drone from automatically disarming after landing (if configured) |
| defer_failsafes                                                     | `bool`   |                                                                  |            | Defer all failsafes that can be deferred (until the flag is cleared)         |
| defer_failsafes_timeout_s | `int16`  |                                                                  |            | Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout    |
| source_type                                                         | `int8`   |                                                                  |            |                                                                                                 |
| source_id                                                           | `uint8`  |                                                                  |            | ID depending on source_type                                                |

## Constants

| 参数名                                                                                                                                | 类型       | 值 | 描述 |
| ---------------------------------------------------------------------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                               | `uint32` | 0 |    |
| <a href="#SOURCE_TYPE_MODE"></a> SOURCE_TYPE_MODE                                        | `int8`   | 0 |    |
| <a href="#SOURCE_TYPE_MODE_EXECUTOR"></a> SOURCE_TYPE_MODE_EXECUTOR | `int8`   | 1 |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                        | `uint8`  | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/ConfigOverridesV0.msg)

:::details
Click here to see original file

```c
# Configurable overrides by (external) modes or mode executors

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)

bool disable_auto_disarm         # Prevent the drone from automatically disarming after landing (if configured)

bool defer_failsafes             # Defer all failsafes that can be deferred (until the flag is cleared)
int16 defer_failsafes_timeout_s  # Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout


int8 SOURCE_TYPE_MODE = 0
int8 SOURCE_TYPE_MODE_EXECUTOR = 1
int8 source_type

uint8 source_id                  # ID depending on source_type

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request
```

:::
