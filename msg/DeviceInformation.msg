# Device information
#
# Can be used to uniquely associate a device_id from a sensor topic with a physical device using serial number.
# as well as tracking of the used firmware versions on the devices.

uint64 timestamp  # time since system start (microseconds)

uint8 device_type  # [@enum DEVICE_TYPE] Type of the device. Matches MAVLink DEVICE_TYPE enum

uint8 DEVICE_TYPE_GENERIC = 0                 # Generic/unknown sensor
uint8 DEVICE_TYPE_AIRSPEED = 1                # Airspeed sensor
uint8 DEVICE_TYPE_ESC = 2                     # ESC
uint8 DEVICE_TYPE_SERVO = 3                   # Servo
uint8 DEVICE_TYPE_GPS = 4                     # GPS
uint8 DEVICE_TYPE_MAGNETOMETER = 5            # Magnetometer
uint8 DEVICE_TYPE_PARACHUTE = 6               # Parachute
uint8 DEVICE_TYPE_RANGEFINDER = 7             # Rangefinder
uint8 DEVICE_TYPE_WINCH = 8                   # Winch
uint8 DEVICE_TYPE_BAROMETER = 9               # Barometer
uint8 DEVICE_TYPE_OPTICAL_FLOW = 10           # Optical flow
uint8 DEVICE_TYPE_ACCELEROMETER = 11          # Accelerometer
uint8 DEVICE_TYPE_GYROSCOPE = 12              # Gyroscope
uint8 DEVICE_TYPE_DIFFERENTIAL_PRESSURE = 13  # Differential pressure
uint8 DEVICE_TYPE_BATTERY = 14                # Battery
uint8 DEVICE_TYPE_HYGROMETER = 15             # Hygrometer

char[32] vendor_name  # Name of the device vendor
char[32] model_name   # Name of the device model

uint32   device_id         # [-] [@invalid 0 if not available] Unique device ID for the sensor. Does not change between power cycles.
char[24] firmware_version  # [-] [@invalid empty if not available] Firmware version.
char[24] hardware_version  # [-] [@invalid empty if not available] Hardware version.
char[33] serial_number     # [-] [@invalid empty if not available] Device serial number or unique identifier.
