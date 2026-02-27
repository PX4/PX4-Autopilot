---
pageClass: is-wide-page
---

# ArmingCheckReplyV0 (UORB message)

**TOPICS:** arming_checkreplyv0

## Fields

| 参数名                                                                                                                 | 类型           | Unit [Frame] | Range/Enum | 描述                                                                       |
| ------------------------------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------ |
| timestamp                                                                                                           | `uint64`     |                                                                  |            | time since system start (microseconds)                |
| request_id                                                                                     | `uint8`      |                                                                  |            |                                                                          |
| registration_id                                                                                | `uint8`      |                                                                  |            |                                                                          |
| health_component_index                                                    | `uint8`      |                                                                  |            | HEALTH_COMPONENT_INDEX_\*                           |
| health_component_is_present                          | `bool`       |                                                                  |            |                                                                          |
| health_component_warning                                                  | `bool`       |                                                                  |            |                                                                          |
| health_component_error                                                    | `bool`       |                                                                  |            |                                                                          |
| can_arm_and_run                                      | `bool`       |                                                                  |            | whether arming is possible, and if it's a navigation mode, if it can run |
| num_events                                                                                     | `uint8`      |                                                                  |            |                                                                          |
| events                                                                                                              | `EventV0[5]` |                                                                  |            |                                                                          |
| mode_req_angular_velocity                            | `bool`       |                                                                  |            |                                                                          |
| mode_req_attitude                                                         | `bool`       |                                                                  |            |                                                                          |
| mode_req_local_alt                                   | `bool`       |                                                                  |            |                                                                          |
| mode_req_local_position                              | `bool`       |                                                                  |            |                                                                          |
| mode_req_local_position_relaxed | `bool`       |                                                                  |            |                                                                          |
| mode_req_global_position                             | `bool`       |                                                                  |            |                                                                          |
| mode_req_mission                                                          | `bool`       |                                                                  |            |                                                                          |
| mode_req_home_position                               | `bool`       |                                                                  |            |                                                                          |
| mode_req_prevent_arming                              | `bool`       |                                                                  |            |                                                                          |
| mode_req_manual_control                              | `bool`       |                                                                  |            |                                                                          |

## Constants

| 参数名                                                                                                                                    | 类型       | 值 | 描述 |
| -------------------------------------------------------------------------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                                   | `uint32` | 0 |    |
| <a href="#HEALTH_COMPONENT_INDEX_NONE"></a> HEALTH_COMPONENT_INDEX_NONE | `uint8`  | 0 |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                            | `uint8`  | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/ArmingCheckReplyV0.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint8 request_id
uint8 registration_id

uint8 HEALTH_COMPONENT_INDEX_NONE = 0

uint8 health_component_index      # HEALTH_COMPONENT_INDEX_*
bool health_component_is_present
bool health_component_warning
bool health_component_error

bool can_arm_and_run              # whether arming is possible, and if it's a navigation mode, if it can run

uint8 num_events

EventV0[5] events

# Mode requirements
bool mode_req_angular_velocity
bool mode_req_attitude
bool mode_req_local_alt
bool mode_req_local_position
bool mode_req_local_position_relaxed
bool mode_req_global_position
bool mode_req_mission
bool mode_req_home_position
bool mode_req_prevent_arming
bool mode_req_manual_control


uint8 ORB_QUEUE_LENGTH = 4
```

:::
