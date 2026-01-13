# Validated airspeed
#
# Provides information about airspeed (indicated, true, calibrated) and the source of the data.
# Used by controllers, estimators and for airspeed reporting to operator.


uint32 MESSAGE_VERSION = 1

uint64 timestamp  # [us] Time since system start

float32 indicated_airspeed_m_s   # [m/s] [@invalid NaN] Indicated airspeed (IAS)
float32 calibrated_airspeed_m_s  # [m/s] [@invalid NaN] Calibrated airspeed (CAS)
float32 true_airspeed_m_s        # [m/s] [@invalid NaN] True airspeed (TAS)

int8 airspeed_source               # [@enum SOURCE] Source of currently published airspeed values
int8 SOURCE_DISABLED = -1          # Disabled
int8 SOURCE_GROUND_MINUS_WIND = 0  # Ground speed minus wind
int8 SOURCE_SENSOR_1 = 1           # Sensor 1
int8 SOURCE_SENSOR_2 = 2           # Sensor 2
int8 SOURCE_SENSOR_3 = 3           # Sensor 3
int8 SOURCE_SYNTHETIC = 4          # Synthetic airspeed 

float32 calibrated_ground_minus_wind_m_s  # [m/s] [@invalid NaN] CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption
float32 calibraded_airspeed_synth_m_s     # [m/s] [@invalid NaN] Synthetic airspeed
float32 airspeed_derivative_filtered      # [m/s^2] Filtered indicated airspeed derivative 
float32 throttle_filtered                 # [-] Filtered fixed-wing throttle 
float32 pitch_filtered                    # [rad] Filtered pitch 
