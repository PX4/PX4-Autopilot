uint64 timestamp		# time since system start (microseconds)

uint8 device_type		# Type of the device (see DEVICE_TYPE enum)
char[32] vendor_name		# Name of the device vendor
char[32] model_name		# Name of the device model

uint32   device_id		# unique device ID for the sensor that does not change between power cycles
char[24] firmware_version	# Version of the firmware. If not available, set to "-1"
char[24] hardware_version	# Version of the hardware. If not available, set to "-1"
char[32] serial_number		# Device serial number or unique identifier. If not available, set to "-1"

# Device type constants matching MAVLink DEVICE_TYPE enum
uint8 DEVICE_TYPE_GENERIC = 0
uint8 DEVICE_TYPE_AIRSPEED = 1
uint8 DEVICE_TYPE_ESC = 2
uint8 DEVICE_TYPE_GPS = 3
uint8 DEVICE_TYPE_MAG = 4
uint8 DEVICE_TYPE_PARACHUTE = 5
uint8 DEVICE_TYPE_RANGEFINDER = 6
uint8 DEVICE_TYPE_WINCH = 7
