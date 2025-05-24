# Arming check reply.
#
# Response to an ArmingCheckRequest message.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # Time since system start [us]

uint8 request_id # Id of ArmingCheckRequest for which this is a response.
uint8 registration_id # Id of component emitting this response.

uint8 HEALTH_COMPONENT_INDEX_NONE = 0 # Index of health component for which this response applies.

uint8 health_component_index # HEALTH_COMPONENT_INDEX_*
bool health_component_is_present ?
bool health_component_warning # ?
bool health_component_error # ?

bool can_arm_and_run # True if arming is possible, and for a navigation mode, if it can run.

uint8 num_events # ?

Event[5] events # ?

# Mode requirements
bool mode_req_angular_velocity # ?
bool mode_req_attitude # Requires an attitude estimate.
bool mode_req_local_alt # Requires a local altitude estimate
bool mode_req_local_position # Requires a local position estimate
bool mode_req_local_position_relaxed # ?
bool mode_req_global_position # Requires a global position estimate
bool mode_req_mission # Requires an uploaded mission
bool mode_req_home_position # Requires a home position
bool mode_req_prevent_arming # ?
bool mode_req_manual_control # Requires a manual controller


uint8 ORB_QUEUE_LENGTH = 4 #
