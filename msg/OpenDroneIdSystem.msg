uint64 timestamp # Local monotonic reception time (microseconds)
uint8 mavlink_channel # Input channel, used to avoid echoing configuration
uint8[20] id_or_mac
uint8 operator_location_type
uint8 classification_type
int32 operator_latitude
int32 operator_longitude
uint16 area_count
uint16 area_radius
float32 area_ceiling
float32 area_floor
uint8 category_eu
uint8 class_eu
float32 operator_altitude_geo
uint32 timestamp_system # Protocol seconds since 2019-01-01
