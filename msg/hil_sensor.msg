# Definition of the hil_sensor uORB topic.

int32 MAGNETOMETER_MODE_NORMAL = 0
int32 MAGNETOMETER_MODE_POSITIVE_BIAS = 1
int32 MAGNETOMETER_MODE_NEGATIVE_BIAS = 2

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
#
int16[3] gyro_raw			# Raw sensor values of angular velocity
float32[3] gyro_rad_s			# Angular velocity in radian per seconds
uint32 gyro_errcount			# Error counter for gyro 0
float32 gyro_temp			# Temperature of gyro 0

int16[3] accelerometer_raw		# Raw acceleration in NED body frame
float32[3] accelerometer_m_s2		# Acceleration in NED body frame, in m/s^2
int16 accelerometer_mode			# Accelerometer measurement mode
float32 accelerometer_range_m_s2		# Accelerometer measurement range in m/s^2
uint64 accelerometer_timestamp	# Accelerometer timestamp
uint32 accelerometer_errcount	# Error counter for accel 0
float32 accelerometer_temp		# Temperature of accel 0

int16[3] magnetometer_raw		# Raw magnetic field in NED body frame
float32[3] magnetometer_ga		# Magnetic field in NED body frame, in Gauss
int16 magnetometer_mode			# Magnetometer measurement mode
float32 magnetometer_range_ga		# measurement range in Gauss
float32 magnetometer_cuttoff_freq_hz	# Internal analog low pass frequency of sensor
uint64 magnetometer_timestamp	# Magnetometer timestamp
uint32 magnetometer_errcount		# Error counter for mag 0
float32 magnetometer_temp		# Temperature of mag 0

int16[3] gyro1_raw			# Raw sensor values of angular velocity
float32[3] gyro1_rad_s			# Angular velocity in radian per seconds
uint64 gyro1_timestamp		# Gyro timestamp
uint32 gyro1_errcount		# Error counter for gyro 1
float32 gyro1_temp			# Temperature of gyro 1

int16[3] accelerometer1_raw		# Raw acceleration in NED body frame
float32[3] accelerometer1_m_s2		# Acceleration in NED body frame, in m/s^2
uint64 accelerometer1_timestamp	# Accelerometer timestamp
uint32 accelerometer1_errcount	# Error counter for accel 1
float32 accelerometer1_temp		# Temperature of accel 1

int16[3] magnetometer1_raw		# Raw magnetic field in NED body frame
float32[3] magnetometer1_ga		# Magnetic field in NED body frame, in Gauss
uint64 magnetometer1_timestamp	# Magnetometer timestamp
uint32 magnetometer1_errcount	# Error counter for mag 1
float32 magnetometer1_temp		# Temperature of mag 1

int16[3] gyro2_raw			# Raw sensor values of angular velocity
float32[3] gyro2_rad_s			# Angular velocity in radian per seconds
uint64 gyro2_timestamp		# Gyro timestamp
uint32 gyro2_errcount		# Error counter for gyro 1
float32 gyro2_temp			# Temperature of gyro 1

int16[3] accelerometer2_raw		# Raw acceleration in NED body frame
float32[3] accelerometer2_m_s2		# Acceleration in NED body frame, in m/s^2
uint64 accelerometer2_timestamp	# Accelerometer timestamp
uint32 accelerometer2_errcount	# Error counter for accel 2
float32 accelerometer2_temp		# Temperature of accel 2

int16[3] magnetometer2_raw		# Raw magnetic field in NED body frame
float32[3] magnetometer2_ga		# Magnetic field in NED body frame, in Gauss
uint64 magnetometer2_timestamp	# Magnetometer timestamp
uint32 magnetometer2_errcount	# Error counter for mag 2
float32 magnetometer2_temp		# Temperature of mag 2

float32 baro_pres_mbar			# Barometric pressure, already temp. comp.
float32 baro_alt_meter			# Altitude, already temp. comp.
float32 baro_temp_celcius		# Temperature in degrees celsius
uint64 baro_timestamp		# Barometer timestamp

float32 baro1_pres_mbar			# Barometric pressure, already temp. comp.
float32 baro1_alt_meter			# Altitude, already temp. comp.
float32 baro1_temp_celcius		# Temperature in degrees celsius
uint64 baro1_timestamp		# Barometer timestamp

float32[10] adc_voltage_v		# ADC voltages of ADC Chan 10/11/12/13 or -1
uint16[10] adc_mapping		# Channel indices of each of these values
float32 mcu_temp_celcius			# Internal temperature measurement of MCU

float32 differential_pressure_pa			# Airspeed sensor differential pressure
uint64 differential_pressure_timestamp	# Last measurement timestamp
float32 differential_pressure_filtered_pa	# Low pass filtered airspeed sensor differential pressure reading

float32 differential_pressure1_pa			# Airspeed sensor differential pressure
uint64 differential_pressure1_timestamp	# Last measurement timestamp
float32 differential_pressure1_filtered_pa	# Low pass filtered airspeed sensor differential pressure reading
