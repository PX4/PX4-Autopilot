# ParameterNameRequest: Request to look up a parameter by name
# Used by external systems (ROS2/DDS) to resolve parameter names to indices

uint64 timestamp                # time since system start (microseconds)

char[17] param_name             # Parameter name (16 chars + null terminator, matches MAVLink)

uint8 ORB_QUEUE_LENGTH = 8
