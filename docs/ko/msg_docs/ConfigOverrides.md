---
pageClass: is-wide-page
---

# ConfigOverrides (UORB message)

Configuration overrides from external modes or mode executors.

An external mode or mode executor requests overrides on config_overrides_request, identifying itself with source_type and source_id.
Commander stores them for that mode or executor and echoes the request on config_overrides_confirm as acknowledgement.
Commander publishes the overrides in effect on config_overrides: those of the current mode, with those of the executor in charge applied on top.

**TOPICS:** config_overrides config_overrides_request config_overrides_confirm

## Fields

| 명칭                                                                                                                                 | 형식       | Unit [Frame] | Range/Enum                                       | 설명                                                                                                                                                                                                                                                                                                                                               |
| ---------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                                                                                | `uint64` | us                                                               |                                                  | Time since system start                                                                                                                                                                                                                                                                                                                          |
| <a id="fld_disable_auto_disarm"></a>disable_auto_disarm                                  | `bool`   |                                                                  |                                                  | Prevent the drone from automatically disarming after landing (if configured)                                                                                                                                                                                                                                                  |
| <a id="fld_defer_failsafes"></a>defer_failsafes                                                               | `bool`   |                                                                  |                                                  | Defer all deferrable failsafes (until the flag is cleared)                                                                                                                                                                                                                                                                    |
| <a id="fld_defer_failsafes_timeout_s"></a>defer_failsafes_timeout_s | `int16`  | s                                                                |                                                  | Maximum time a failsafe can be deferred. 0 = system default. (Invalid: -1 No timeout)                                                                                                                                                                                         |
| <a id="fld_disable_auto_set_home"></a>disable_auto_set_home         | `bool`   |                                                                  |                                                  | Prevent the drone from automatically setting the home position on arm or takeoff                                                                                                                                                                                                                                                                 |
| <a id="fld_source_type"></a>source_type                                                                       | `int8`   |                                                                  | [SOURCE_TYPE](#SOURCE_TYPE) | Origin of this override                                                                                                                                                                                                                                                                                                                          |
| <a id="fld_source_id"></a>source_id                                                                           | `uint8`  |                                                                  |                                                  | ID assigned at registration (RegisterExtComponentReply): mode_id for SOURCE_TYPE_MODE, mode_executor_id for SOURCE_TYPE_MODE_EXECUTOR |

## Enums

### SOURCE_TYPE {#SOURCE_TYPE}

Used in field(s): [source_type](#fld_source_type)

| 명칭                                                                                                                               | 형식     | Value | 설명                                       |
| -------------------------------------------------------------------------------------------------------------------------------- | ------ | ----- | ---------------------------------------- |
| <a id="#SOURCE_TYPE_MODE"></a> SOURCE_TYPE_MODE                                        | `int8` | 0     | Override originates from a mode          |
| <a id="#SOURCE_TYPE_MODE_EXECUTOR"></a> SOURCE_TYPE_MODE_EXECUTOR | `int8` | 1     | Override originates from a mode executor |

## Constants

| 명칭                                                                                        | 형식       | Value | 설명 |
| ----------------------------------------------------------------------------------------- | -------- | ----- | -- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION                        | `uint32` | 1     |    |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8`  | 4     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ConfigOverrides.msg)

:::details
Click here to see original file

```c
# Configuration overrides from external modes or mode executors
#
# An external mode or mode executor requests overrides on config_overrides_request, identifying itself with source_type and source_id.
# Commander stores them for that mode or executor and echoes the request on config_overrides_confirm as acknowledgement.
# Commander publishes the overrides in effect on config_overrides: those of the current mode, with those of the executor in charge applied on top.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start

bool disable_auto_disarm # Prevent the drone from automatically disarming after landing (if configured)

bool defer_failsafes # Defer all deferrable failsafes (until the flag is cleared)
int16 defer_failsafes_timeout_s # [s] [@invalid -1 No timeout] Maximum time a failsafe can be deferred. 0 = system default.
bool disable_auto_set_home # Prevent the drone from automatically setting the home position on arm or takeoff

int8 source_type # [@enum SOURCE_TYPE] Origin of this override
int8 SOURCE_TYPE_MODE = 0 # Override originates from a mode
int8 SOURCE_TYPE_MODE_EXECUTOR = 1 # Override originates from a mode executor

uint8 source_id # [-] ID assigned at registration (RegisterExtComponentReply): mode_id for SOURCE_TYPE_MODE, mode_executor_id for SOURCE_TYPE_MODE_EXECUTOR

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS config_overrides config_overrides_request config_overrides_confirm
```

:::
