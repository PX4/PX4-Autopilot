---
pageClass: is-wide-page
---

# ArmingCheckReplyV0 (UORB message)

**TOPICS:** arming_check_reply_v0

## Fields

| 参数名                                                                                                                                                                 | 类型           | Unit [Frame] | Range/Enum | 描述                                                                       |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                                                                                                                 | `uint64`     |                                                                  |            | time since system start (microseconds)                |
| <a id="fld_request_id"></a>request_id                                                                                                          | `uint8`      |                                                                  |            |                                                                          |
| <a id="fld_registration_id"></a>registration_id                                                                                                | `uint8`      |                                                                  |            |                                                                          |
| <a id="fld_health_component_index"></a>health_component_index                                                             | `uint8`      |                                                                  |            | HEALTH_COMPONENT_INDEX_\*                           |
| <a id="fld_health_component_is_present"></a>health_component_is_present                              | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_health_component_warning"></a>health_component_warning                                                         | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_health_component_error"></a>health_component_error                                                             | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_can_arm_and_run"></a>can_arm_and_run                                                      | `bool`       |                                                                  |            | whether arming is possible, and if it's a navigation mode, if it can run |
| <a id="fld_num_events"></a>num_events                                                                                                          | `uint8`      |                                                                  |            |                                                                          |
| <a id="fld_events"></a>events                                                                                                                                       | `EventV0[5]` |                                                                  |            |                                                                          |
| <a id="fld_mode_req_angular_velocity"></a>mode_req_angular_velocity                                  | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_attitude"></a>mode_req_attitude                                                                       | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_local_alt"></a>mode_req_local_alt                                                | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_local_position"></a>mode_req_local_position                                      | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_local_position_relaxed"></a>mode_req_local_position_relaxed | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_global_position"></a>mode_req_global_position                                    | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_mission"></a>mode_req_mission                                                                         | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_home_position"></a>mode_req_home_position                                        | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_prevent_arming"></a>mode_req_prevent_arming                                      | `bool`       |                                                                  |            |                                                                          |
| <a id="fld_mode_req_manual_control"></a>mode_req_manual_control                                      | `bool`       |                                                                  |            |                                                                          |

## Constants

| 参数名                                                                                                                                  | 类型       | 值 | 描述 |
| ------------------------------------------------------------------------------------------------------------------------------------ | -------- | - | -- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                                   | `uint32` | 0 |    |
| <a id="#HEALTH_COMPONENT_INDEX_NONE"></a> HEALTH_COMPONENT_INDEX_NONE | `uint8`  | 0 |    |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                            | `uint8`  | 4 |    |

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
