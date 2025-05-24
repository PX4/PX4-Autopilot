# Arming check request
#
# Broadcast message to request arming checks be reported by all registered components, such as internal and external modes.
# An ArmingCheckReply response should be published by all registered components.
# The reply will include the published request_id, allowing correlation of all arming check information.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # Time since system start [us]

uint8 request_id # Id of this request. Allows correlation with associated ArmingCheckReply messages.
