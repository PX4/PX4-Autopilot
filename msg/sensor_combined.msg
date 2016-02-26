# Definition of the sensor_combined uORB topic.

int32 MAGNETOMETER_MODE_NORMAL = 0
int32 MAGNETOMETER_MODE_POSITIVE_BIAS = 1
int32 MAGNETOMETER_MODE_NEGATIVE_BIAS = 2

uint32 SENSOR_PRIO_MIN = 0
uint32 SENSOR_PRIO_VERY_LOW = 25
uint32 SENSOR_PRIO_LOW = 50
uint32 SENSOR_PRIO_DEFAULT = 75
uint32 SENSOR_PRIO_HIGH = 100
uint32 SENSOR_PRIO_VERY_HIGH = 125
uint32 SENSOR_PRIO_MAX = 255

# Sensor readings in raw and SI-unit form.
#
# These values are read from the sensors. Raw values are in sensor-specific units,
# the scaled values are in SI-units, as visible from the ending of the variable
# or the comments. The use of the SI fields is in general advised, as these fields
# are scaled and offset-compensated where possible and do not change with board
# revisions and sensor updates.
#
# Actual data, this is specific to the type of data which is stored in this struct
# A line containing L0GME will be added by the Python logging code generator to the logged dataset.
#
# NOTE: Ordering of fields optimized to align to 32 bit / 4 bytes Change with consideration only

uint64 timestamp			# Timestamp in microseconds since boot, from gyro
uint64[3] gyro_timestamp			# Gyro timestamps
int16[9] gyro_raw			# Raw sensor values of angular velocity
float32[9] gyro_rad_s			# Angular velocity in radian per seconds
uint32[3] gyro_priority			# Sensor priority
float32[9] gyro_integral_rad		# delta angle in radians
uint64[3] gyro_integral_dt			# delta time for gyro integral in us
uint32[3] gyro_errcount			# Error counter for gyro 0
float32[3] gyro_temp			# Temperature of gyro 0

int16[9] accelerometer_raw		# Raw acceleration in NED body frame
float32[9] accelerometer_m_s2		# Acceleration in NED body frame, in m/s^2
float32[9] accelerometer_integral_m_s		# velocity in NED body frame, in m/s^2
uint64[3] accelerometer_integral_dt		# delta time for accel integral in us
int16[3] accelerometer_mode			# Accelerometer measurement mode
float32[3] accelerometer_range_m_s2		# Accelerometer measurement range in m/s^2
uint64[3] accelerometer_timestamp	# Accelerometer timestamp
uint32[3] accelerometer_priority	# Sensor priority
uint32[3] accelerometer_errcount	# Error counter for accel 0
float32[3] accelerometer_temp		# Temperature of accel 0

int16[9] magnetometer_raw		# Raw magnetic field in NED body frame
float32[9] magnetometer_ga		# Magnetic field in NED body frame, in Gauss
int16[3] magnetometer_mode			# Magnetometer measurement mode
float32[3] magnetometer_range_ga		# measurement range in Gauss
float32[3] magnetometer_cuttoff_freq_hz	# Internal analog low pass frequency of sensor
uint64[3] magnetometer_timestamp	# Magnetometer timestamp
uint32[3] magnetometer_priority			# Sensor priority
uint32[3] magnetometer_errcount		# Error counter for mag 0
float32[3] magnetometer_temp		# Temperature of mag 0

float32[3] baro_pres_mbar			# Barometric pressure, already temp. comp.
float32[3] baro_alt_meter			# Altitude, already temp. comp.
float32[3] baro_temp_celcius		# Temperature in degrees celsius
uint64[3] baro_timestamp		# Barometer timestamp
uint32[3] baro_priority			# Sensor priority
uint32[3] baro_errcount			# Error count in communication

float32[10] adc_voltage_v		# ADC voltages of ADC Chan 10/11/12/13 or -1
uint16[10] adc_mapping		# Channel indices of each of these values
float32 mcu_temp_celcius			# Internal temperature measurement of MCU

float32[3] differential_pressure_pa			# Airspeed sensor differential pressure
uint64[3] differential_pressure_timestamp	# Last measurement timestamp
float32[3] differential_pressure_filtered_pa	# Low pass filtered airspeed sensor differential pressure reading
uint32[3] differential_pressure_priority			# Sensor priority
uint32[3] differential_pressure_errcount			# Error count in communication
