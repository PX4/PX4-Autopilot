---
pageClass: is-wide-page
---

# ConfigOverrides (повідомлення UORB)

Configurable overrides by (external) modes or mode executors.

**TOPICS:** config_overrides config_overrides_request config_overrides_confirm

## Fields

| Назва                                                                                                                              | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                            |
| ---------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                                                                | `uint64` |                                                                  |            | time since system start (microseconds)                                       |
| <a id="fld_disable_auto_disarm"></a>disable_auto_disarm                                  | `bool`   |                                                                  |            | Prevent the drone from automatically disarming after landing (if configured) |
| <a id="fld_defer_failsafes"></a>defer_failsafes                                                               | `bool`   |                                                                  |            | Defer all failsafes that can be deferred (until the flag is cleared)         |
| <a id="fld_defer_failsafes_timeout_s"></a>defer_failsafes_timeout_s | `int16`  |                                                                  |            | Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout    |
| <a id="fld_disable_auto_set_home"></a>disable_auto_set_home         | `bool`   |                                                                  |            | Prevent the drone from automatically setting the home position on arm or takeoff                |
| <a id="fld_source_type"></a>source_type                                                                       | `int8`   |                                                                  |            |                                                                                                 |
| <a id="fld_source_id"></a>source_id                                                                           | `uint8`  |                                                                  |            | ID depending on source_type                                                |

## Constants

| Назва                                                                                                                            | Тип      | Значення | Опис |
| -------------------------------------------------------------------------------------------------------------------------------- | -------- | -------- | ---- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                               | `uint32` | 1        |      |
| <a id="#SOURCE_TYPE_MODE"></a> SOURCE_TYPE_MODE                                        | `int8`   | 0        |      |
| <a id="#SOURCE_TYPE_MODE_EXECUTOR"></a> SOURCE_TYPE_MODE_EXECUTOR | `int8`   | 1        |      |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                        | `uint8`  | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ConfigOverrides.msg)

:::details
Click here to see original file

```c
# Configurable overrides by (external) modes or mode executors

uint32 MESSAGE_VERSION = 1

uint64 timestamp		# time since system start (microseconds)

bool disable_auto_disarm         # Prevent the drone from automatically disarming after landing (if configured)

bool defer_failsafes             # Defer all failsafes that can be deferred (until the flag is cleared)
int16 defer_failsafes_timeout_s  # Maximum time a failsafe can be deferred. 0 = system default, -1 = no timeout
bool disable_auto_set_home       # Prevent the drone from automatically setting the home position on arm or takeoff

int8 SOURCE_TYPE_MODE = 0
int8 SOURCE_TYPE_MODE_EXECUTOR = 1
int8 source_type

uint8 source_id                  # ID depending on source_type

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request config_overrides_confirm
```

:::
