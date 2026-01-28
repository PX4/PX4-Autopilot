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
