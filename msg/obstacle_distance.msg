# Obstacle distances in front of the sensor.
uint64 timestamp		# time since system start (microseconds)

uint8 sensor_type # Type from MAV_DISTANCE_SENSOR enum.
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
uint8 MAV_DISTANCE_SENSOR_RADAR = 3

uint16[72] distances # Distance of obstacles in front of the sensor starting on the left side. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstace is present. A value of UINT16_MAX for unknown/not used. In a array element, each unit corresponds to 1cm.

uint8 increment # Angular width in degrees of each array element.

uint16 min_distance # Minimum distance the sensor can measure in centimeters.
uint16 max_distance # Maximum distance the sensor can measure in centimeters.
