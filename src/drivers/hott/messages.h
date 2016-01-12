/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file messages.h
 * @author Simon Wilks <sjwilks@gmail.com>
 *
 * Graupner HoTT Telemetry message generation.
 *
 */
#ifndef MESSAGES_H_
#define MESSAGES_H

#include <stdlib.h>


void build_gam_response(uint8_t *buffer, size_t *size);
void build_eam_response(uint8_t *buffer, size_t *size);
void build_alt_response(uint8_t *buffer, size_t *size);
void build_gps_response(uint8_t *buffer, size_t *size);
void build_esc_response(uint8_t *buffer, size_t *size);

void convert_to_degrees_minutes_seconds(double val, int *deg, int *min, int *sec);
void init_sub_messages(void);
void init_pub_messages(void);
void build_gam_request(uint8_t *buffer, size_t *size);
void publish_gam_message(const uint8_t *buffer);

/* The HoTT receiver demands a minimum 5ms period of silence after delivering its request.
 * Note that the value specified here is lower than 5000 (5ms) as time is lost constucting
 * the message after the read which takes some milliseconds.
 */
#define POST_READ_DELAY_IN_USECS	4000
/* A pause of 3ms is required between each uint8_t sent back to the HoTT receiver. Much lower
 * values can be used in practise though.
 */
#define POST_WRITE_DELAY_IN_USECS	2500

#define POLL_TIMEOUT_IN_MSECS		3500

/* The GAM Module poll message. */
struct gam_module_poll_msg {
	uint8_t mode;
	uint8_t id;
};



/* Protocol constants. */
#define BINARY_REQUEST_ID		0x80
#define TEXT_REQUEST_ID			0x7f

#define START_BYTE				0x7c
#define STOP_BYTE				0x7d

#define BINARY_MESSAGE_LEN		45
#define TEXT_MESSAGE_LEN		173


/* Sensor IDs */
#define GAM_SENSOR_ID			0x8d
#define GAM_SENSOR_TEXT_ID		0xd0

#define EAM_SENSOR_ID			0x8e
#define EAM_SENSOR_TEXT_ID		0xe0

#define ALT_SENSOR_ID			0x89
#define ALT_SENSOR_TEXT_ID		0x90

#define GPS_SENSOR_ID			0x8a
#define GPS_SENSOR_TEXT_ID		0xa0

#define ESC_SENSOR_ID			0x8c
#define ESC_SENSOR_TEXT_ID		0xc0



#define TEXT_LEN_TXT			168
#define TEXT_START_BYTE			0x7b
#define TEXT_FILL_BYTE			0x00

/** Text mode message */
struct text_mode_msg {
	int8_t start;					/**< TEXT_START_BYTE */
	int8_t fill;					/**< TEXT_FILL_BYTE */
	int8_t warning;					/**< 1=A 2=B ... */
	int8_t msg_txt[TEXT_LEN_TXT];	/**< ASCII text to display, Bit 7 = 1 -> Inverse character display, Display 21x8 */
	int8_t stop_byte;				/**< STOP_BYTE */
	int8_t parity;					/**< Lower 8-bits of all bytes summed */
};	/* 173 bytes */



/** The Electric Air Module message. */
struct eam_module_msg {
	uint8_t start;					/**< START_BYTE */
	uint8_t eam_sensor_id;			/**< EAM_SENSOR_ID	*/
	uint8_t warning;				/**< 1=A 2=B ... */
	uint8_t sensor_text_id;			/**< EAM_SENSOR_TEXT_ID */
	uint8_t alarm_inverse1;
	uint8_t alarm_inverse2;
	uint8_t cell1_L;				/**< Lipo cell voltages. 0.02V steps, 124=2.48V	*/
	uint8_t cell2_L;
	uint8_t cell3_L;
	uint8_t cell4_L;
	uint8_t cell5_L;
	uint8_t cell6_L;
	uint8_t cell7_L;
	uint8_t cell1_H;
	uint8_t cell2_H;
	uint8_t cell3_H;
	uint8_t cell4_H;
	uint8_t cell5_H;
	uint8_t cell6_H;
	uint8_t cell7_H;
	uint8_t batt1_voltage_L;		/**< Battery 1 voltage, lower 8-bits in steps of 0.02V 	*/
	uint8_t batt1_voltage_H;
	uint8_t batt2_voltage_L;		/**< Battery 2 voltage, lower 8-bits in steps of 0.02V 	*/
	uint8_t batt2_voltage_H;
	uint8_t temperature1;			/**< Temperature sensor 1. 20 = 0 degrees */
	uint8_t temperature2;
	uint8_t altitude_L;				/**< Attitude (meters) lower 8-bits. 500 = 0 meters */
	uint8_t altitude_H;
	uint8_t current_L;				/**< Current (mAh) lower 8-bits in steps of 0.1V */
	uint8_t current_H;
	uint8_t main_voltage_L;			/**< Main power voltage lower 8-bits in steps of 0.1V */
	uint8_t main_voltage_H;
	uint8_t battery_capacity_L;		/**< Used battery capacity in steps of 10mAh */
	uint8_t battery_capacity_H;
	uint8_t climbrate_L;			/**< Climb rate in 0.01m/s. 0m/s = 30000 */
	uint8_t climbrate_H;
	uint8_t climbrate_3s;			/**< Climb rate in m/3sec. 0m/3sec = 120 */
	uint8_t rpm_L;					/**< RPM Lower 8-bits In steps of 10 U/min */
	uint8_t rpm_H;
	uint8_t electric_min;			/**< Flight time in minutes. */
	uint8_t electric_sec;			/**< Flight time in seconds. */
	uint8_t speed_L;				/**< Airspeed in km/h in steps of 1 km/h */
	uint8_t speed_H;
	uint8_t stop;					/**< STOP_BYTE */
	uint8_t checksum;				/**< Lower 8-bits of all bytes summed. */
}; /* 45 bytes */



/**
 * General Air Module (GAM) message
 * Warning values :
'Q' Min cell voltage sensor 1
'R' Min Battery 1 voltage sensor 1
'J' Max Battery 1 voltage sensor 1
'F' Min temperature sensor 1
'H' Max temperature sensor 1
'S' Min Battery 2 voltage sensor 2
'K' Max Battery 2 voltage sensor 2
'G' Min temperature sensor 2
'I' Max temperature sensor 2
'W' Max current
'V' Max capacity mAh
'P' Min main power voltage
'X' Max main power voltage
'O' Min altitude
'Z' Max altitude
'C' negative difference m/s too high
'A' negative difference m/3s too high
'N' positive difference m/s too high
'L' positive difference m/3s too high
'T' Minimum RPM
'Y' Maximum RPM
*/
struct gam_module_msg {
	uint8_t start;					/**< START_BYTE	*/
	uint8_t gam_sensor_id;			/**< GAM_SENSOR_ID */
	uint8_t warning;				/**< 1=A 2=B ... */
	uint8_t sensor_text_id;			/**< GAM_SENSOR_TEXT_ID */
	uint8_t alarm_invers1;			/**< alarm bitmask. Value is displayed inverted
										Bit#	Alarm field
										0		all cell voltage
										1		Battery 1
										2		Battery 2
										3		Temperature 1
										4		Temperature 2
										5		Fuel
										6		mAh
										7		Altitude
									*/
	uint8_t alarm_invers2;			/**< alarm bitmask. Value is displayed inverted
										Bit#	Alarm Field
										0		main power current
										1		main power voltage
										2		Altitude
										3		m/s
										4		m/3s
										5		unknown
										6		unknown
										7		"ON" sign/text msg active
									*/
	uint8_t cell1;					/**< Lipo cell voltages. 0.02V steps, 124=2.48V */
	uint8_t cell2;
	uint8_t cell3;
	uint8_t cell4;
	uint8_t cell5;
	uint8_t cell6;
	uint8_t batt1_L;				/**< Battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V	*/
	uint8_t batt1_H;
	uint8_t batt2_L;				/**< Battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V	*/
	uint8_t batt2_H;
	uint8_t temperature1;			/**< Temperature 1. offset of 20. a value of 20 = 0°C */
	uint8_t temperature2;			/**< Temperature 2. offset of 20. a value of 20 = 0°C */
	uint8_t fuel_percent;			/**< Fuel capacity in %. Values 0 - 100, Graphical display ranges: 0 25% 50% 75% 100% */
	uint8_t fuel_ml_L;				/**< Fuel in ml scale. Full = 65535, graphical display ranges: 0-25% 50% 75% 100% */
	uint8_t fuel_ml_H;
	uint8_t rpm_L;					/**< RPM in 10 RPM steps. 300 = 3000rpm */
	uint8_t rpm_H;
	uint8_t altitude_L;				/**< Altitude in meters. offset of 500, 500 = 0m */
	uint8_t altitude_H;
	uint8_t climbrate_L;			/**< Climb rate in 0.01m/s. Value of 30000 = 0.00 m/s */
	uint8_t climbrate_H;
	uint8_t climbrate3s;			/**< Climb rate in m/3sec. Value of 120 = 0m/3sec */
	uint8_t current_L;				/**< Current in 0.1A steps */
	uint8_t current_H;
	uint8_t main_voltage_L;			/**< Main power voltage using 0.1V steps */
	uint8_t main_voltage_H;
	uint8_t batt_cap_L;				/**< Used battery capacity in 10mAh steps */
	uint8_t batt_cap_H;
	uint8_t speed_L;				/**< Speed in km/h */
	uint8_t speed_H;
	uint8_t min_cell_volt;			/**< Minimum cell voltage in 2mV steps. 124 = 2.48V */
	uint8_t min_cell_volt_num;		/**< Number of the cell with the lowest voltage */
	uint8_t rpm2_L;					/**< RPM in 10 RPM steps. 300 = 3000rpm */
	uint8_t rpm2_H;
	uint8_t general_error_number;	/**< Voice error == 12. TODO: more docu */
	uint8_t pressure;				/**< Pressure up to 16bar. 0.1bar scale. 20 = 2bar */
	uint8_t version;
	uint8_t stop;					/**< STOP_BYTE */
	uint8_t checksum;				/**< Lower 8-bits of all bytes summed */
}; /* 45 bytes */



/**
 * The GPS sensor message
 * Struct based on: https://code.google.com/p/diy-hott-gps/downloads
 * Warning values :
'A' Min Speed
'L' Max Speed
'O' Min Altitude
'Z' Max Altitude
'C' (negative) sink rate m/sec to high
'B' (negative) sink rate m/3sec to high
'N' climb rate m/sec to high
'M' climb rate m/3sec to high
'D' Max home distance
 */
struct gps_module_msg {
	uint8_t start;					/**< START_BYTE */
	uint8_t sensor_id;				/**< GPS_SENSOR_ID */
	uint8_t warning;				/**< 1=A 2=B ... */
	uint8_t sensor_text_id;			/**< GPS_SENSOR_TEXT_ID */
	uint8_t alarm_inverse1;			/**< 01 inverse status */
	uint8_t alarm_inverse2;			/**< 00 inverse status status 1 = no GPS Signal */
	uint8_t flight_direction;		/**< 119 = Flightdir./dir. 1 = 2°; 0° (North), 90° (East), 180° (South), 270° (West) */
	uint8_t gps_speed_L;			/**< 8 = /GPS speed low byte 8km/h */
	uint8_t gps_speed_H;			/**< 0 = /GPS speed high byte */

	uint8_t latitude_ns;			/**< 000 = N = 48°39’988 */
	uint8_t latitude_min_L;			/**< 231 0xE7 = 0x12E7 = 4839 */
	uint8_t latitude_min_H;			/**< 018 18 = 0x12 */
	uint8_t latitude_sec_L;			/**< 171 220 = 0xDC = 0x03DC =0988 */
	uint8_t latitude_sec_H;			/**< 016 3 = 0x03 */

	uint8_t longitude_ew;			/**< 000  = E= 9° 25’9360 */
	uint8_t longitude_min_L;		/**< 150 157 = 0x9D = 0x039D = 0925 */
	uint8_t longitude_min_H;		/**< 003 3 = 0x03 */
	uint8_t longitude_sec_L;		/**< 056 144 = 0x90 0x2490 = 9360 */
	uint8_t longitude_sec_H;		/**< 004 36 = 0x24 */

	uint8_t distance_L;				/**< 027 123 = /distance low byte 6 = 6 m */
	uint8_t distance_H;				/**< 036 35 = /distance high byte */
	uint8_t altitude_L;				/**< 243 244 = /Altitude low byte 500 = 0m */
	uint8_t altitude_H;				/**< 001 1 = /Altitude high byte */
	uint8_t resolution_L;			/**< 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
	uint8_t resolution_H;			/**< 117 = High Byte m/s resolution 0.01m */
	uint8_t unknown1;				/**< 120 = 0m/3s */
	uint8_t gps_num_sat;			/**< GPS.Satellites (number of satelites) (1 byte) */
	uint8_t gps_fix_char;			/**< GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
	uint8_t home_direction;			/**< HomeDirection (direction from starting point to Model position) (1 byte) */
	uint8_t angle_x_direction;		/**< angle x-direction (1 byte) */
	uint8_t angle_y_direction;		/**< angle y-direction (1 byte) */
	uint8_t angle_z_direction;		/**< angle z-direction (1 byte) */
	uint8_t gyro_x_L;				/**< gyro x low byte (2 bytes) */
	uint8_t gyro_x_H;				/**< gyro x high byte */
	uint8_t gyro_y_L;				/**< gyro y low byte (2 bytes) */
	uint8_t gyro_y_H;				/**< gyro y high byte */
	uint8_t gyro_z_L;				/**< gyro z low byte (2 bytes) */
	uint8_t gyro_z_H;				/**< gyro z high byte */
	uint8_t vibration;				/**< vibration (1 bytes) */
	uint8_t ascii4;					/**< 00 ASCII Free Character [4] */
	uint8_t ascii5;					/**< 00 ASCII Free Character [5] */
	uint8_t gps_fix;				/**< 00 ASCII Free Character [6], we use it for GPS FIX */
	uint8_t version;
	uint8_t stop;					/**< STOP_BYTE */
	uint8_t checksum;				/**< Lower 8-bits of all bytes summed */
};



#define ALT_LEN_TXT				21

/** Altimeter sensor msg
 * Warning Values :
'Q' Min cell voltage sensor 1
'R' Min Battery 1 voltage sensor 1
'J' Max Battery 1 voltage sensor 1
'F' Min temperature sensor 1
'H' Max temperature sensor 1
'S' Min Battery voltage sensor 2
'K' Max Battery voltage sensor 2
'G' Min temperature sensor 2
'I' Max temperature sensor 2
'W' Max current
'V' Max capacity mAh
'P' Min main power voltage
'X' Max main power voltage
'O' Min altitude
'Z' Max altitude
'T' Minimum RPM
'Y' Maximum RPM
'C' m/s negative difference
'A' m/3s negative difference
*/
struct alt_module_msg {
	int8_t start;					/**< START_BYTE */
	int8_t alt_sensor_id;			/**< ALT_SENSOR_ID */
	int8_t warning;					/**< TODO: figure out codes */
	int8_t sensor_text_id;			/**< ALT_SENSOR_TEXT_ID */
	int8_t alarm_invers1;			/**< Inverse display (alarm?) bitmask */
	int8_t altitude_L;				/**< Altitude low int8_t. In meters. A value of 500 means 0m */
	int8_t altitude_H;				/**< Altitude high int8_t */
	int8_t altitude_max_L;			/**< Max. measured altitude low int8_t. In meters. A value of 500 means 0m */
	int8_t altitude_max_H;			/**< Max. measured altitude high int8_t */
	int8_t altitude_min_L;			/**< Min. measured altitude low int8_t. In meters. A value of 500 means 0m */
	int8_t altitude_min_H;			/**< Min. measured altitude high int8_t */
	int8_t climbrate_L;				/**< Climb rate in m/s. Steps of 0.01m/s. Value of 30000 = 0.00 m/s */
	int8_t climbrate_H;				/**< Climb rate in m/s */
	int8_t climbrate3s_L;			/**< Climb rate in m/3s. Steps of 0.01m/3s. Value of 30000 = 0.00 m/3s */
	int8_t climbrate3s_H;			/**< Climb rate m/3s low int8_t */
	int8_t climbrate10s_L;			/**< Climb rate m/10s. Steps of 0.01m/10s. Value of 30000 = 0.00 m/10s */
	int8_t climbrate10s_H;			/**< Climb rate m/10s low int8_t */
	int8_t text_msg[ALT_LEN_TXT];	/**< Free ASCII text message */
	int8_t free_char1;				/**< Free ASCII character.  appears right to home distance */
	int8_t free_char2;				/**< Free ASCII character.  appears right to home direction */
	int8_t free_char3;				/**< Free ASCII character.  appears? TODO: Check where this char appears */
	int8_t compass_direction;		/**< Compass heading in 2° steps. 1 = 2° */
	int8_t version;					/**< version number TODO: more info? */
	int8_t stop;					/**< STOP_BYTE */
	int8_t checksum;				/**< Lower 8-bits of all bytes summed */
};


/** Air ESC Module
 * Warning values :
'A' ?
'L' ?
'O' ?
'Z' ?
'C' ?
'B' ?
'N' ?
'M' ?
'D' ?
*/
struct esc_module_msg {
	uint8_t start;					/**< START_BYTE */
	uint8_t esc_sensor_id;			/**< ESC_SENSOR_ID */
	uint8_t warning;				/**< 1=A 2=B ... */
	uint8_t sensor_text_id;			/**< ESC_SENSOR_TEXT_ID */
	uint8_t alarm_invers1;			/**< TODO: more info */
	uint8_t alarm_invers2;			/**< TODO: more info */
	uint8_t input_v_L;				/**< Input voltage low byte */
	uint8_t input_v_H;
	uint8_t input_v_min_L;			/**< Input min. voltage low byte */
	uint8_t input_v_min_H;
	uint8_t batt_cap_L;				/**< battery capacity in 10mAh steps */
	uint8_t batt_cap_H;
	uint8_t	esc_temp;				/**< ESC temperature */
	uint8_t esc_max_temp;			/**< ESC max. temperature */
	uint8_t current_L;				/**< Current in 0.1 steps */
	uint8_t current_H;
	uint8_t	current_max_L;			/**< Current max. in 0.1 steps */
	uint8_t current_max_H;
	uint8_t rpm_L;					/**< RPM in 10U/min steps */
	uint8_t	rpm_H;
	uint8_t	rpm_max_L;				/**< RPM max */
	uint8_t rpm_max_H;
	uint8_t throttle;				/**< throttle in % */
	uint8_t speed_L;				/**< Speed */
	uint8_t speed_H;
	uint8_t	speed_max_L;			/**< Speed max */
	uint8_t speed_max_H;
	uint8_t bec_v;					/**< BEC voltage */
	uint8_t bec_min_v;				/**< BEC min. voltage */
	uint8_t bec_current;			/**< BEC current */
	uint8_t bec_current_max_L;		/**< BEC max. current */
	uint8_t bec_current_max_H;		/**< TODO: not really clear why 2 bytes... */
	uint8_t pwm;					/**< PWM */
	uint8_t bec_temp;				/**< BEC temperature */
	uint8_t bec_temp_max;			/**< BEC highest temperature */
	uint8_t motor_temp;				/**< Motor or external sensor temperature */
	uint8_t motor_temp_max;			/**< Highest motor or external sensor temperature */
	uint8_t motor_rpm_L;			/**< Motor or external RPM sensor (without gear) */
	uint8_t motor_rpm_H;
	uint8_t motor_timing;			/**< Motor timing */
	uint8_t motor_timing_adv;		/**< Motor advanced timing */
	uint8_t motor_highest_current;	/**< Motor number (1-x) with highest current */
	uint8_t version;				/**< Version number (highest current motor 1-x) */
	uint8_t stop;					/**< STOP_BYTE */
	uint8_t checksum;				/**< Lower 8-bits of all bytes summed */
}; /* 45 bytes */

#endif /* MESSAGES_H_ */
