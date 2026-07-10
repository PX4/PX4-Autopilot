uint64 timestamp	# time since system start (microseconds)

uint32 device_id

bool heater_on
bool temperature_target_met
bool temperature_activation_threshold_met

float32 temperature_sensor
float32 temperature_target

uint32 controller_period_usec
uint32 controller_time_on_usec

float32 proportional_value
float32 integrator_value
float32 feed_forward_value

float32 supply_voltage		# Supply voltage (V)
float32 heater_current		# Heater current (A)
float32 nominal_multiplier

uint8 MODE_GPIO  = 1
uint8 MODE_PX4IO = 2
uint8 mode

uint8 TEMPERATURE_SOURCE_IMU   = 0
uint8 TEMPERATURE_SOURCE_HYGRO = 1
uint8 temperature_source
