uint64 timestamp # time since system start (microseconds)

# Request to PX4 to get the hash of a message, to check for message compatibility

uint16 LATEST_PROTOCOL_VERSION = 1 # Current version of this protocol. Increase this whenever the MessageFormatRequest or MessageFormatResponse changes.

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command
