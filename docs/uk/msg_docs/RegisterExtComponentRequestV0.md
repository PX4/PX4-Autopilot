---
pageClass: is-wide-page
---

# RegisterExtComponentRequestV0 (UORB message)

Request to register an external component.

**TOPICS:** register_extcomponent_requestv0

## Fields

| Назва                                                                                       | Тип        | Unit [Frame] | Range/Enum | Опис                                                                                                                       |
| ------------------------------------------------------------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                   | `uint64`   |                                                                  |            | time since system start (microseconds)                                                                  |
| request_id                                                             | `uint64`   |                                                                  |            | ID, set this to a random value                                                                                             |
| name                                                                                        | `char[25]` |                                                                  |            | either the requested mode name, or component name                                                                          |
| px4_ros2_api_version         | `uint16`   |                                                                  |            | Set to LATEST_PX4_ROS2_API_VERSION     |
| register_arming_check                             | `bool`     |                                                                  |            |                                                                                                                            |
| register_mode                                                          | `bool`     |                                                                  |            | registering a mode also requires arming_check to be set                                               |
| register_mode_executor                            | `bool`     |                                                                  |            | registering an executor also requires a mode to be registered (which is the owned mode by the executor) |
| enable_replace_internal_mode | `bool`     |                                                                  |            | set to true if an internal mode should be replaced                                                                         |
| replace_internal_mode                             | `uint8`    |                                                                  |            | vehicle_status::NAVIGATION_STATE_\*                                   |
| activate_mode_immediately                         | `bool`     |                                                                  |            | switch to the registered mode (can only be set in combination with an executor)                         |

## Constants

| Назва                                                                                                                                                       | Тип      | Значення | Опис                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                                                        | `uint32` | 0        |                                                                                                                                                                                                                        |
| <a href="#LATEST_PX4_ROS2_API_VERSION"></a> LATEST_PX4_ROS2_API_VERSION | `uint16` | 1        | API version compatibility. Increase this on a breaking semantic change. Changes to any message field are detected separately and do not require an API version change. |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                                                 | `uint8`  | 2        |                                                                                                                                                                                                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/px4_msgs_old/msg/RegisterExtComponentRequestV0.msg)

:::details
Click here to see original file

```c
# Request to register an external component

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint64 request_id                  # ID, set this to a random value
char[25] name                      # either the requested mode name, or component name

uint16 LATEST_PX4_ROS2_API_VERSION = 1 # API version compatibility. Increase this on a breaking semantic change. Changes to any message field are detected separately and do not require an API version change.

uint16 px4_ros2_api_version   # Set to LATEST_PX4_ROS2_API_VERSION

# Components to be registered
bool register_arming_check
bool register_mode                 # registering a mode also requires arming_check to be set
bool register_mode_executor        # registering an executor also requires a mode to be registered (which is the owned mode by the executor)

bool enable_replace_internal_mode  # set to true if an internal mode should be replaced
uint8 replace_internal_mode        # vehicle_status::NAVIGATION_STATE_*
bool activate_mode_immediately     # switch to the registered mode (can only be set in combination with an executor)


uint8 ORB_QUEUE_LENGTH = 2
```

:::
