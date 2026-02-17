# Arming check request
#
# Broadcast message to request arming checks be reported by all registered components, such as external ROS 2 navigation modes.
# All registered components should respond with an ArmingCheckReply message that indicates their current mode requirements, and any arming failure information.
# The request is sent regularly, even while armed, so that the FMU always knows the current arming state for external modes, and can forward it to ground stations.
#
# The reply will include the published request_id, allowing correlation of all arming check information for a particular request.
# The reply will also include the registration_id for each external component, provided to it during the registration process (RegisterExtComponentReply).

uint32 MESSAGE_VERSION = 1

uint64 timestamp  # [us] Time since system start

uint8 request_id  # [-] Id of this request. Allows correlation with associated ArmingCheckReply messages.

uint32 valid_registrations_mask # [-] Bitmask of valid registration ID's (the bit is also cleared if flagged as unresponsive)
