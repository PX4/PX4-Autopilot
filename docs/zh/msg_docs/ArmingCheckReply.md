---
pageClass: is-wide-page
---

# ArmingCheckReply (UORB message)

Arming check reply.

This is a response to an ArmingCheckRequest message sent by the FMU to an external component, such as a ROS 2 navigation mode.
The response contains the current set of external mode requirements, and a queue of events indicating recent failures to set the mode (which the FMU may then forward to a ground station).
The request is sent regularly to all registered ROS modes, even while armed, so that the FMU always knows and can forward the current state.

Note that the external component is identified by its registration_id, which is allocated to the component during registration (arming_check_id in RegisterExtComponentReply).
The message is not used by internal/FMU components, as their mode requirements are known at compile time.

**TOPICS:** arming_checkreply

## Fields

| 参数名                                                                                                                  | 类型         | Unit [Frame] | Range/Enum                                                                                  | 描述                                                                                                                                                                                     |
| -------------------------------------------------------------------------------------------------------------------- | ---------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                            | `uint64`   | us                                                               |                                                                                             | Time since system start.                                                                                                                                               |
| request_id                                                                                      | `uint8`    |                                                                  |                                                                                             | Id of ArmingCheckRequest for which this is a response                                                                                                                                  |
| registration_id                                                                                 | `uint8`    |                                                                  |                                                                                             | Id of external component emitting this response                                                                                                                                        |
| health_component_index                                                     | `uint8`    |                                                                  | [HEALTH_COMPONENT_INDEX](#HEALTH_COMPONENT_INDEX) |                                                                                                                                                                                        |
| health_component_is_present                           | `bool`     |                                                                  |                                                                                             | Unused. Intended for use with health events interface (health_component_t in events.json) |
| health_component_warning                                                   | `bool`     |                                                                  |                                                                                             | Unused. Intended for use with health events interface (health_component_t in events.json) |
| health_component_error                                                     | `bool`     |                                                                  |                                                                                             | Unused. Intended for use with health events interface (health_component_t in events.json) |
| can_arm_and_run                                       | `bool`     |                                                                  |                                                                                             | True if the component can arm. For navigation mode components, true if the component can arm in the mode or switch to the mode when already armed                      |
| num_events                                                                                      | `uint8`    |                                                                  |                                                                                             | Number of queued failure messages (Event) in the events field                                                                                                       |
| events                                                                                                               | `Event[5]` |                                                                  |                                                                                             | Arming failure reasons (Queue of events to report to GCS)                                                                                                           |
| mode_req_angular_velocity                             | `bool`     |                                                                  |                                                                                             | Requires angular velocity estimate (e.g. from gyroscope)                                                                            |
| mode_req_attitude                                                          | `bool`     |                                                                  |                                                                                             | Requires an attitude estimate                                                                                                                                                          |
| mode_req_local_alt                                    | `bool`     |                                                                  |                                                                                             | Requires a local altitude estimate                                                                                                                                                     |
| mode_req_local_position                               | `bool`     |                                                                  |                                                                                             | Requires a local position estimate                                                                                                                                                     |
| mode_req_local_position_relaxed  | `bool`     |                                                                  |                                                                                             | Requires a more relaxed global position estimate                                                                                                                                       |
| mode_req_global_position                              | `bool`     |                                                                  |                                                                                             | Requires a global position estimate                                                                                                                                                    |
| mode_req_global_position_relaxed | `bool`     |                                                                  |                                                                                             | Requires a relaxed global position estimate                                                                                                                                            |
| mode_req_mission                                                           | `bool`     |                                                                  |                                                                                             | Requires an uploaded mission                                                                                                                                                           |
| mode_req_home_position                                | `bool`     |                                                                  |                                                                                             | Requires a home position (such as RTL/Return mode)                                                                                                                  |
| mode_req_prevent_arming                               | `bool`     |                                                                  |                                                                                             | Prevent arming (such as in Land mode)                                                                                                                               |
| mode_req_manual_control                               | `bool`     |                                                                  |                                                                                             | Requires a manual controller                                                                                                                                                           |

## Enums

### HEALTH_COMPONENT_INDEX {#HEALTH_COMPONENT_INDEX}

| 参数名                                                                                                                                    | 类型      | 值 | 描述                                                        |
| -------------------------------------------------------------------------------------------------------------------------------------- | ------- | - | --------------------------------------------------------- |
| <a href="#HEALTH_COMPONENT_INDEX_NONE"></a> HEALTH_COMPONENT_INDEX_NONE | `uint8` | 0 | Index of health component for which this response applies |

## Constants

| 参数名                                                                                         | 类型       | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                        | `uint32` | 1 |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8`  | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ArmingCheckReply.msg)

:::details
Click here to see original file

```c
# Arming check reply
#
# This is a response to an ArmingCheckRequest message sent by the FMU to an external component, such as a ROS 2 navigation mode.
# The response contains the current set of external mode requirements, and a queue of events indicating recent failures to set the mode (which the FMU may then forward to a ground station).
# The request is sent regularly to all registered ROS modes, even while armed, so that the FMU always knows and can forward the current state.
#
# Note that the external component is identified by its registration_id, which is allocated to the component during registration (arming_check_id in RegisterExtComponentReply).
# The message is not used by internal/FMU components, as their mode requirements are known at compile time.

uint32 MESSAGE_VERSION  = 1

uint64 timestamp # [us] Time since system start.

uint8 request_id       # [-] Id of ArmingCheckRequest for which this is a response
uint8 registration_id  # [-] Id of external component emitting this response

uint8 HEALTH_COMPONENT_INDEX_NONE = 0  # Index of health component for which this response applies

uint8 health_component_index      # [@enum HEALTH_COMPONENT_INDEX]
bool health_component_is_present  # Unused. Intended for use with health events interface (health_component_t in events.json)
bool health_component_warning     # Unused. Intended for use with health events interface (health_component_t in events.json)
bool health_component_error       # Unused. Intended for use with health events interface (health_component_t in events.json)

bool can_arm_and_run  # True if the component can arm. For navigation mode components, true if the component can arm in the mode or switch to the mode when already armed

uint8 num_events  # Number of queued failure messages (Event) in the events field

Event[5] events  # Arming failure reasons (Queue of events to report to GCS)

# Mode requirements
bool mode_req_angular_velocity         # Requires angular velocity estimate (e.g. from gyroscope)
bool mode_req_attitude                 # Requires an attitude estimate
bool mode_req_local_alt                # Requires a local altitude estimate
bool mode_req_local_position           # Requires a local position estimate
bool mode_req_local_position_relaxed   # Requires a more relaxed global position estimate
bool mode_req_global_position          # Requires a global position estimate
bool mode_req_global_position_relaxed  # Requires a relaxed global position estimate
bool mode_req_mission                  # Requires an uploaded mission
bool mode_req_home_position            # Requires a home position (such as RTL/Return mode)
bool mode_req_prevent_arming           # Prevent arming (such as in Land mode)
bool mode_req_manual_control           # Requires a manual controller

uint8 ORB_QUEUE_LENGTH  = 4
```

:::
