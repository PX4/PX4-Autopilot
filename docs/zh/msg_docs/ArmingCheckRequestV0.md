---
pageClass: is-wide-page
---

# ArmingCheckRequestV0 (UORB message)

Arming check request.

Broadcast message to request arming checks be reported by all registered components, such as external ROS 2 navigation modes.
All registered components should respond with an ArmingCheckReply message that indicates their current mode requirements, and any arming failure information.
The request is sent regularly, even while armed, so that the FMU always knows the current arming state for external modes, and can forward it to ground stations.

The reply will include the published request_id, allowing correlation of all arming check information for a particular request.
The reply will also include the registration_id for each external component, provided to it during the registration process (RegisterExtComponentReply).

**TOPICS:** arming_checkrequestv0

## Fields

| 参数名                             | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                                                |
| ------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------- |
| timestamp                       | `uint64` | us                                                               |            | Time since system start.                                                                          |
| request_id | `uint8`  |                                                                  |            | Id of this request. Allows correlation with associated ArmingCheckReply messages. |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/ArmingCheckRequestV0.msg)

:::details
Click here to see original file

```c
# Arming check request.
#
# Broadcast message to request arming checks be reported by all registered components, such as external ROS 2 navigation modes.
# All registered components should respond with an ArmingCheckReply message that indicates their current mode requirements, and any arming failure information.
# The request is sent regularly, even while armed, so that the FMU always knows the current arming state for external modes, and can forward it to ground stations.
#
# The reply will include the published request_id, allowing correlation of all arming check information for a particular request.
# The reply will also include the registration_id for each external component, provided to it during the registration process (RegisterExtComponentReply).

uint32 MESSAGE_VERSION = 0

uint64 timestamp  # [us] Time since system start.

uint8 request_id  # Id of this request. Allows correlation with associated ArmingCheckReply messages.
```

:::
