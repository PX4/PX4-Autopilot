# ArmingCheckReply (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ArmingCheckReply.msg)

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

Event[5] events

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
