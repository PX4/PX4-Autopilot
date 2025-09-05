uint64 timestamp		# time since system start (microseconds)

uint8 device_type		# Type of the device (see DEVICE_TYPE enum)
char[16] vendor_name		# Name of the device vendor
char[16] model_name		# Name of the device model
char[32] firmware_version	# Version of the firmware. If not available, set to "-1"
char[32] hardware_version	# Version of the hardware. If not available, set to "-1"
char[32] serial_number		# Device serial number or unique identifier. If not available, set to "-1"

# Device type constants matching MAVLink DEVICE_TYPE enum
uint8 DEVICE_TYPE_GENERIC = 0
uint8 DEVICE_TYPE_GPS = 1
uint8 DEVICE_TYPE_MAG = 2
uint8 DEVICE_TYPE_AIRSPEED = 3
uint8 DEVICE_TYPE_RANGEFINDER = 4
uint8 DEVICE_TYPE_ESC = 5
