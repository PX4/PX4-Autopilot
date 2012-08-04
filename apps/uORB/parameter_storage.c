
#include "parameter_storage.h"
#include <stdbool.h>


/* Global symbols / flags */

struct global_data_parameter_storage_t global_data_parameter_storage_d =  { /*.counter = 0, .timestamp = 0,*/ .pm = {.size = PARAM_MAX_COUNT,
			.param_values[PARAM_SYSTEM_ID] = 12,
			.param_names[PARAM_SYSTEM_ID] = "SYS_ID",
			.param_needs_write[PARAM_SYSTEM_ID] = false,

			.param_values[PARAM_COMP_ID] = 200,
			.param_names[PARAM_COMP_ID] = "COMP_ID",
			.param_needs_write[PARAM_COMP_ID] = false,

			.param_values[PARAM_FLIGHT_ENV] = (float)PX4_FLIGHT_ENVIRONMENT_INDOOR,
			.param_names[PARAM_FLIGHT_ENV] = "FLIGHT_ENV",
			.param_needs_write[PARAM_FLIGHT_ENV] = false,

			.param_values[PARAM_BATTERYVOLTAGE_CONVERSION] = -1.f,
			.param_names[PARAM_BATTERYVOLTAGE_CONVERSION] = "BATVOLTAG_CONV",
			.param_needs_write[PARAM_BATTERYVOLTAGE_CONVERSION] = false,

			.param_values[PARAM_PID_YAWPOS_P] = 0.3f,
			.param_names[PARAM_PID_YAWPOS_P] = "PID_YAWPOS_P",
			.param_needs_write[PARAM_PID_YAWPOS_P] = false,

			.param_values[PARAM_PID_YAWPOS_I] = 0.15f,
			.param_names[PARAM_PID_YAWPOS_I] = "PID_YAWPOS_I",
			.param_needs_write[PARAM_PID_YAWPOS_I] = false,

			.param_values[PARAM_PID_YAWPOS_D] = 0.0f,
			.param_names[PARAM_PID_YAWPOS_D] = "PID_YAWPOS_D",
			.param_needs_write[PARAM_PID_YAWPOS_D] = false,

			.param_values[PARAM_PID_YAWPOS_AWU] = 1.0f,
			.param_names[PARAM_PID_YAWPOS_AWU] = "PID_YAWPOS_AWU",
			.param_needs_write[PARAM_PID_YAWPOS_AWU] = false,

			.param_values[PARAM_PID_YAWPOS_LIM] = 3.0f,
			.param_names[PARAM_PID_YAWPOS_LIM] = "PID_YAWPOS_LIM",
			.param_needs_write[PARAM_PID_YAWPOS_LIM] = false,

			.param_values[PARAM_PID_YAWSPEED_P] = 0.1f,
			.param_names[PARAM_PID_YAWSPEED_P] = "PID_YAWSPD_P",
			.param_needs_write[PARAM_PID_YAWSPEED_P] = false,

			.param_values[PARAM_PID_YAWSPEED_I] = 0.02f,
			.param_names[PARAM_PID_YAWSPEED_I] = "PID_YAWSPD_I",
			.param_needs_write[PARAM_PID_YAWSPEED_I] = false,

			.param_values[PARAM_PID_YAWSPEED_D] = 0.0f,
			.param_names[PARAM_PID_YAWSPEED_D] = "PID_YAWSPD_D",
			.param_needs_write[PARAM_PID_YAWSPEED_D] = false,

			.param_values[PARAM_PID_YAWSPEED_AWU] = 0.02f,
			.param_names[PARAM_PID_YAWSPEED_AWU] = "PID_YAWSPD_AWU",
			.param_needs_write[PARAM_PID_YAWSPEED_AWU] = false,

			.param_values[PARAM_PID_YAWSPEED_LIM] = 0.1f,
			.param_names[PARAM_PID_YAWSPEED_LIM] = "PID_YAWSPD_LIM",
			.param_needs_write[PARAM_PID_YAWSPEED_LIM] = false,

			.param_values[PARAM_PID_ATT_P] = 0.3f,
			.param_names[PARAM_PID_ATT_P] = "PID_ATT_P",
			.param_needs_write[PARAM_PID_ATT_P] = false,

			.param_values[PARAM_PID_ATT_I] = 0.0f,
			.param_names[PARAM_PID_ATT_I] = "PID_ATT_I",
			.param_needs_write[PARAM_PID_ATT_I] = false,

			.param_values[PARAM_PID_ATT_D] = 0.1f,
			.param_names[PARAM_PID_ATT_D] = "PID_ATT_D",
			.param_needs_write[PARAM_PID_ATT_D] = false,

			.param_values[PARAM_PID_ATT_AWU] = 0.05f,
			.param_names[PARAM_PID_ATT_AWU] = "PID_ATT_AWU",
			.param_needs_write[PARAM_PID_ATT_AWU] = false,

			.param_values[PARAM_PID_ATT_LIM] = 0.3f,
			.param_names[PARAM_PID_ATT_LIM] = "PID_ATT_LIM",
			.param_needs_write[PARAM_PID_ATT_LIM] = false,

			.param_values[PARAM_PID_POS_P] = 40.0f,
			.param_names[PARAM_PID_POS_P] = "PID_POS_P",
			.param_needs_write[PARAM_PID_POS_P] = false,

			.param_values[PARAM_PID_POS_I] = 0.0f,
			.param_names[PARAM_PID_POS_I] = "PID_POS_I",
			.param_needs_write[PARAM_PID_POS_I] = false,

			.param_values[PARAM_PID_POS_D] = 0.0f,
			.param_names[PARAM_PID_POS_D] = "PID_POS_D",
			.param_needs_write[PARAM_PID_POS_D] = false,

			.param_values[PARAM_PID_POS_AWU] = 5.0f,
			.param_names[PARAM_PID_POS_AWU] = "PID_POS_AWU",
			.param_needs_write[PARAM_PID_POS_AWU] = false,

			.param_values[PARAM_PID_POS_LIM] = 0.3f,
			.param_names[PARAM_PID_POS_LIM] = "PID_POS_LIM",
			.param_needs_write[PARAM_PID_POS_LIM] = false,

			.param_values[PARAM_PID_POS_Z_P] = 10.0f,
			.param_names[PARAM_PID_POS_Z_P] = "PID_POS_Z_P",
			.param_needs_write[PARAM_PID_POS_Z_P] = false,

			.param_values[PARAM_PID_POS_Z_I] = 0.0f,
			.param_names[PARAM_PID_POS_Z_I] = "PID_POS_Z_I",
			.param_needs_write[PARAM_PID_POS_Z_I] = false,

			.param_values[PARAM_PID_POS_Z_D] = 0.0f,
			.param_names[PARAM_PID_POS_Z_D] = "PID_POS_Z_D",
			.param_needs_write[PARAM_PID_POS_Z_D] = false,

			.param_values[PARAM_PID_POS_Z_AWU] = 3.0f,
			.param_names[PARAM_PID_POS_Z_AWU] = "PID_POS_Z_AWU",
			.param_needs_write[PARAM_PID_POS_Z_AWU] = false,

			.param_values[PARAM_PID_POS_Z_LIM] = 0.3f,
			.param_names[PARAM_PID_POS_Z_LIM] = "PID_POS_Z_LIM",
			.param_needs_write[PARAM_PID_POS_Z_LIM] = false,

			.param_values[PARAM_AIRSPEED] = 30.0f,
			.param_names[PARAM_AIRSPEED] = "AIRSPEED",
			.param_needs_write[PARAM_AIRSPEED] = false,

			.param_values[PARAM_WPLON] = -120.0f,
			.param_names[PARAM_WPLON] = "WPLON",
			.param_needs_write[PARAM_WPLON] = false,

			.param_values[PARAM_WPLAT] = 38.0f,
			.param_names[PARAM_WPLAT] = "WPLAT",
			.param_needs_write[PARAM_WPLAT] = false,

			.param_values[PARAM_WPALT] = 500.0f,
			.param_names[PARAM_WPALT] = "WPALT",
			.param_needs_write[PARAM_WPALT] = false,

			.param_values[PARAM_FLIGHTMODE] = CRUISE,
			.param_names[PARAM_FLIGHTMODE] = "FLIGHTMODE",
			.param_needs_write[PARAM_FLIGHTMODE] = false,

			.param_values[PARAM_SENSOR_GYRO_XOFFSET] = 700.f,
			.param_names[PARAM_SENSOR_GYRO_XOFFSET] = "SENSOR_GYRO_XOF",
			.param_needs_write[PARAM_SENSOR_GYRO_XOFFSET] = false,

			.param_values[PARAM_SENSOR_GYRO_YOFFSET] = 1400.0f,
			.param_names[PARAM_SENSOR_GYRO_YOFFSET] = "SENSOR_GYRO_YOF",
			.param_needs_write[PARAM_SENSOR_GYRO_YOFFSET] = false,

			.param_values[PARAM_SENSOR_GYRO_ZOFFSET] = 0.0f,
			.param_names[PARAM_SENSOR_GYRO_ZOFFSET] = "SENSOR_GYRO_ZOF",
			.param_needs_write[PARAM_SENSOR_GYRO_ZOFFSET] = false,

			.param_values[PARAM_SENSOR_MAG_XOFFSET] = 422.0f,
			.param_names[PARAM_SENSOR_MAG_XOFFSET] = "SENSOR_MAG_XOF",
			.param_needs_write[PARAM_SENSOR_MAG_XOFFSET] = false,

			.param_values[PARAM_SENSOR_MAG_YOFFSET] = -85.0f,
			.param_names[PARAM_SENSOR_MAG_YOFFSET] = "SENSOR_MAG_YOF",
			.param_needs_write[PARAM_SENSOR_MAG_YOFFSET] = false,

			.param_values[PARAM_SENSOR_MAG_ZOFFSET] = -370.0f,
			.param_names[PARAM_SENSOR_MAG_ZOFFSET] = "SENSOR_MAG_ZOF",
			.param_needs_write[PARAM_SENSOR_MAG_ZOFFSET] = false,

			.param_values[PARAM_ATT_XOFFSET] = 0.0f,
			.param_names[PARAM_ATT_XOFFSET] = "ATT_XOFF",
			.param_needs_write[PARAM_ATT_XOFFSET] = false,

			.param_values[PARAM_ATT_YOFFSET] = 0.0f,
			.param_names[PARAM_ATT_YOFFSET] = "ATT_YOFF",
			.param_needs_write[PARAM_ATT_YOFFSET] = false,

			.param_values[PARAM_RC1_MIN] = 1000.0f,
			.param_names[PARAM_RC1_MIN] = "RC1_MIN",
			.param_needs_write[PARAM_RC1_MIN] = false,

			.param_values[PARAM_RC1_MAX] = 2000.0f,
			.param_names[PARAM_RC1_MAX] = "RC1_MAX",
			.param_needs_write[PARAM_RC1_MAX] = false,

			.param_values[PARAM_RC1_TRIM] = 1500.0f,
			.param_names[PARAM_RC1_TRIM] = "RC1_TRIM",
			.param_needs_write[PARAM_RC1_TRIM] = false,

			.param_values[PARAM_RC1_REV] = 1.0f,
			.param_names[PARAM_RC1_REV] = "RC1_REV",
			.param_needs_write[PARAM_RC1_REV] = false,

			.param_values[PARAM_RC2_MIN] = 1000.0f,
			.param_names[PARAM_RC2_MIN] = "RC2_MIN",
			.param_needs_write[PARAM_RC2_MIN] = false,

			.param_values[PARAM_RC2_MAX] = 2000.0f,
			.param_names[PARAM_RC2_MAX] = "RC2_MAX",
			.param_needs_write[PARAM_RC2_MAX] = false,

			.param_values[PARAM_RC2_TRIM] = 1500.0f,
			.param_names[PARAM_RC2_TRIM] = "RC2_TRIM",
			.param_needs_write[PARAM_RC2_TRIM] = false,

			.param_values[PARAM_RC2_REV] = 1.0f,
			.param_names[PARAM_RC2_REV] = "RC2_REV",
			.param_needs_write[PARAM_RC2_REV] = false,

			.param_values[PARAM_RC3_MIN] = 1000.0f,
			.param_names[PARAM_RC3_MIN] = "RC3_MIN",
			.param_needs_write[PARAM_RC3_MIN] = false,

			.param_values[PARAM_RC3_MAX] = 2000.0f,
			.param_names[PARAM_RC3_MAX] = "RC3_MAX",
			.param_needs_write[PARAM_RC3_MAX] = false,

			.param_values[PARAM_RC3_TRIM] = 1500.0f,
			.param_names[PARAM_RC3_TRIM] = "RC3_TRIM",
			.param_needs_write[PARAM_RC3_TRIM] = false,

			.param_values[PARAM_RC3_REV] = 1.0f,
			.param_names[PARAM_RC3_REV] = "RC3_REV",
			.param_needs_write[PARAM_RC3_REV] = false,

			.param_values[PARAM_RC4_MIN] = 1000.0f,
			.param_names[PARAM_RC4_MIN] = "RC4_MIN",
			.param_needs_write[PARAM_RC4_MIN] = false,

			.param_values[PARAM_RC4_MAX] = 2000.0f,
			.param_names[PARAM_RC4_MAX] = "RC4_MAX",
			.param_needs_write[PARAM_RC4_MAX] = false,

			.param_values[PARAM_RC4_TRIM] = 1500.0f,
			.param_names[PARAM_RC4_TRIM] = "RC4_TRIM",
			.param_needs_write[PARAM_RC4_TRIM] = false,

			.param_values[PARAM_RC4_REV] = 1.0f,
			.param_names[PARAM_RC4_REV] = "RC4_REV",
			.param_needs_write[PARAM_RC4_MIN] = false,

			.param_values[PARAM_RC5_MIN] = 1000.0f,
			.param_names[PARAM_RC5_MIN] = "RC5_MIN",
			.param_needs_write[PARAM_RC5_MIN] = false,

			.param_values[PARAM_RC5_MAX] = 2000.0f,
			.param_names[PARAM_RC5_MAX] = "RC5_MAX",
			.param_needs_write[PARAM_RC5_MAX] = false,

			.param_values[PARAM_RC5_TRIM] = 1500.0f,
			.param_names[PARAM_RC5_TRIM] = "RC5_TRIM",
			.param_needs_write[PARAM_RC5_TRIM] = false,

			.param_values[PARAM_RC5_REV] = 1.0f,
			.param_names[PARAM_RC5_REV] = "RC5_REV",
			.param_needs_write[PARAM_RC5_REV] = false,

			.param_values[PARAM_RC6_MIN] = 1000.0f,
			.param_names[PARAM_RC6_MIN] = "RC6_MIN",
			.param_needs_write[PARAM_RC6_MIN] = false,

			.param_values[PARAM_RC6_MAX] = 2000.0f,
			.param_names[PARAM_RC6_MAX] = "RC6_MAX",
			.param_needs_write[PARAM_RC6_MAX] = false,

			.param_values[PARAM_RC6_TRIM] = 1500.0f,
			.param_names[PARAM_RC6_TRIM] = "RC6_TRIM",
			.param_needs_write[PARAM_RC6_TRIM] = false,

			.param_values[PARAM_RC6_REV] = 1.0f,
			.param_names[PARAM_RC6_REV] = "RC6_REV",
			.param_needs_write[PARAM_RC6_REV] = false,

			.param_values[PARAM_RC7_MIN] = 1000,
			.param_names[PARAM_RC7_MIN] = "RC7_MIN",
			.param_needs_write[PARAM_RC7_MIN] = false,

			.param_values[PARAM_RC7_MAX] = 2000,
			.param_names[PARAM_RC7_MAX] = "RC7_MAX",
			.param_needs_write[PARAM_RC7_MAX] = false,

			.param_values[PARAM_RC7_TRIM] = 1500,
			.param_names[PARAM_RC7_TRIM] = "RC7_TRIM",
			.param_needs_write[PARAM_RC7_TRIM] = false,

			.param_values[PARAM_RC7_REV] = 1.0f,
			.param_names[PARAM_RC7_REV] = "RC7_REV",
			.param_needs_write[PARAM_RC7_REV] = false,

			.param_values[PARAM_RC8_MIN] = 1000,
			.param_names[PARAM_RC8_MIN] = "RC8_MIN",
			.param_needs_write[PARAM_RC8_MIN] = false,

			.param_values[PARAM_RC8_MAX] = 2000,
			.param_names[PARAM_RC8_MAX] = "RC8_MAX",
			.param_needs_write[PARAM_RC8_MAX] = false,

			.param_values[PARAM_RC8_TRIM] = 1500,
			.param_names[PARAM_RC8_TRIM] = "RC8_TRIM",
			.param_needs_write[PARAM_RC8_TRIM] = false,

			.param_values[PARAM_RC8_REV] = 1.0f,
			.param_names[PARAM_RC8_REV] = "RC8_REV",
			.param_needs_write[PARAM_RC8_REV] = false,

			.param_values[PARAM_ROLL_CHAN] = 1,
			.param_names[PARAM_ROLL_CHAN] = "ROLL_CHAN",
			.param_needs_write[PARAM_ROLL_CHAN] = false,

			.param_values[PARAM_PITCH_CHAN] = 2,
			.param_names[PARAM_PITCH_CHAN] = "PITCH_CHAN",
			.param_needs_write[PARAM_PITCH_CHAN] = false,

			.param_values[PARAM_THROTTLE_CHAN] = 3,
			.param_names[PARAM_THROTTLE_CHAN] = "THROTTLE_CHAN",
			.param_needs_write[PARAM_THROTTLE_CHAN] = false,

			.param_values[PARAM_YAW_CHAN] = 4,
			.param_names[PARAM_YAW_CHAN] = "YAW_CHAN",
			.param_needs_write[PARAM_YAW_CHAN] = false,

			.param_values[PARAM_OVERRIDE_CHAN] = 5,
			.param_names[PARAM_OVERRIDE_CHAN] = "OVERRIDE_CHAN",
			.param_needs_write[PARAM_OVERRIDE_CHAN] = false,

			.param_values[PARAM_SERVO1_MIN] = 1000.0f,
			.param_names[PARAM_SERVO1_MIN] = "SERVO1_MIN",
			.param_needs_write[PARAM_SERVO1_MIN] = false,

			.param_values[PARAM_SERVO1_MAX] = 2000.0f,
			.param_names[PARAM_SERVO1_MAX] = "SERVO1_MAX",
			.param_needs_write[PARAM_SERVO1_MAX] = false,

			.param_values[PARAM_SERVO1_TRIM] = 1500.0f,
			.param_names[PARAM_SERVO1_TRIM] = "SERVO1_TRIM",
			.param_needs_write[PARAM_SERVO1_TRIM] = false,

			.param_values[PARAM_SERVO2_MIN] = 1000.0f,
			.param_names[PARAM_SERVO2_MIN] = "SERVO2_MIN",
			.param_needs_write[PARAM_SERVO2_MIN] = false,

			.param_values[PARAM_SERVO2_MAX] = 2000.0f,
			.param_names[PARAM_SERVO2_MAX] = "SERVO2_MAX",
			.param_needs_write[PARAM_SERVO2_MAX] = false,

			.param_values[PARAM_SERVO2_TRIM] = 1500.0f,
			.param_names[PARAM_SERVO2_TRIM] = "SERVO2_TRIM",
			.param_needs_write[PARAM_SERVO2_TRIM] = false,

			.param_values[PARAM_SERVO3_MIN] = 1000.0f,
			.param_names[PARAM_SERVO3_MIN] = "SERVO3_MIN",
			.param_needs_write[PARAM_SERVO3_MIN] = false,

			.param_values[PARAM_SERVO3_MAX] = 2000.0f,
			.param_names[PARAM_SERVO3_MAX] = "SERVO3_MAX",
			.param_needs_write[PARAM_SERVO3_MAX] = false,

			.param_values[PARAM_SERVO3_TRIM] = 1500.0f,
			.param_names[PARAM_SERVO3_TRIM] = "SERVO3_TRIM",
			.param_needs_write[PARAM_SERVO3_TRIM] = false,

			.param_values[PARAM_SERVO4_MIN] = 1000.0f,
			.param_names[PARAM_SERVO4_MIN] = "SERVO4_MIN",
			.param_needs_write[PARAM_SERVO4_MIN] = false,

			.param_values[PARAM_SERVO4_MAX] = 2000.0f,
			.param_names[PARAM_SERVO4_MAX] = "SERVO4_MAX",
			.param_needs_write[PARAM_SERVO4_MAX] = false,

			.param_values[PARAM_SERVO4_TRIM] = 1500.0f,
			.param_names[PARAM_SERVO4_TRIM] = "SERVO4_TRIM",
			.param_needs_write[PARAM_SERVO4_TRIM] = false,

			.param_values[PARAM_SERVO_SCALE] = 20.0f,
			.param_names[PARAM_SERVO_SCALE] = "SERVO_SCALE",
			.param_needs_write[PARAM_SERVO_SCALE] = false
															    }
};

struct global_data_parameter_storage_t *global_data_parameter_storage = &global_data_parameter_storage_d;
