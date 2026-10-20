uint64 timestamp # Local monotonic reception time (microseconds)
uint8 mavlink_channel # Input channel, used to avoid echoing configuration
uint8[20] id_or_mac
uint8 operator_id_type
char[20] operator_id
