uint64 timestamp		# time since system start (microseconds)

# modules
uint8 MODULE_MAINBRAIN = 2
uint8 MODULE_POWER = 3
uint8 MODULE_BUOYANCY = 4
uint8 MODULE_HYDRAULIC = 5

uint8 SENSOR_HUMIDITY = 0
uint8 SENSOR_TEMPERATURE = 1
uint8 SENSOR_PRESSURE = 2

uint8 module # see MODULE_*
uint8 sensor # see SENSOR_*
float32 value # sensor value

