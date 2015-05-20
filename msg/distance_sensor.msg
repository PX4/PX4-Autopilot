# DISTANCE_SENSOR message data

uint32 time_boot_ms

uint16 min_distance
uint16 max_distance
uint16 current_distance

uint8 type
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2

uint8 id

uint8 orientation
uint8 covariance