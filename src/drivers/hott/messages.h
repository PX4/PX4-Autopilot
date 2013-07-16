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

/* The HoTT receiver demands a minimum 5ms period of silence after delivering its request.
 * Note that the value specified here is lower than 5000 (5ms) as time is lost constucting
 * the message after the read which takes some milliseconds.
 */
#define POST_READ_DELAY_IN_USECS	4000
/* A pause of 3ms is required between each uint8_t sent back to the HoTT receiver. Much lower
 * values can be used in practise though.
 */
#define POST_WRITE_DELAY_IN_USECS	2000

// Protocol constants.
#define BINARY_MODE_REQUEST_ID	0x80	// Binary mode request.
#define START_BYTE		0x7c
#define STOP_BYTE		0x7d
#define TEMP_ZERO_CELSIUS	0x14

/* The GAM Module poll message. */
struct gam_module_poll_msg {
	uint8_t mode;
	uint8_t id;
};

/* Electric Air Module (EAM) constants. */
#define EAM_SENSOR_ID			0x8e
#define EAM_SENSOR_TEXT_ID		0xe0

/* The Electric Air Module message. */
struct eam_module_msg {
	uint8_t start;			/**< Start byte   				*/
	uint8_t eam_sensor_id;		/**< EAM sensor					*/
	uint8_t warning;
	uint8_t sensor_text_id;
	uint8_t alarm_inverse1;
	uint8_t alarm_inverse2;
	uint8_t cell1_L;		/**< Lipo cell voltages. Not supported.	*/
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
	uint8_t batt1_voltage_L;	/**< Battery 1 voltage, lower 8-bits in steps of 0.02V 	*/
	uint8_t batt1_voltage_H;
	uint8_t batt2_voltage_L;	/**< Battery 2 voltage, lower 8-bits in steps of 0.02V 	*/
	uint8_t batt2_voltage_H;
	uint8_t temperature1;		/**< Temperature sensor 1. 20 = 0 degrees */
	uint8_t temperature2;
	uint8_t altitude_L;		/**< Attitude (meters) lower 8-bits. 500 = 0 meters */
	uint8_t altitude_H;
	uint8_t current_L;		/**< Current (mAh) lower 8-bits in steps of 0.1V */
	uint8_t current_H;
	uint8_t main_voltage_L;		/**< Main power voltage lower 8-bits in steps of 0.1V */
	uint8_t main_voltage_H;
	uint8_t battery_capacity_L;	/**< Used battery capacity in steps of 10mAh */
	uint8_t battery_capacity_H;
	uint8_t climbrate_L;		/**< Climb rate in 0.01m/s. 0m/s = 30000 */
	uint8_t climbrate_H;
	uint8_t climbrate_3s;		/**< Climb rate in m/3sec. 0m/3sec = 120 */
	uint8_t rpm_L;			/**< RPM Lower 8-bits In steps of 10 U/min */
	uint8_t rpm_H;
	uint8_t electric_min;		/**< Flight time in minutes. */
	uint8_t electric_sec;		/**< Flight time in seconds. */
	uint8_t speed_L;		/**< Airspeed in km/h in steps of 1 km/h */
	uint8_t speed_H;
	uint8_t stop;			/**< Stop byte */
	uint8_t checksum;		/**< Lower 8-bits of all bytes summed. */
};


/* General Air Module (GAM) constants. */
#define GAM_SENSOR_ID			0x8d
#define GAM_SENSOR_TEXT_ID		0xd0

struct gam_module_msg {
        uint8_t start;		// start byte constant value 0x7c
        uint8_t gam_sensor_id;		// EAM sensort id. constat value 0x8d
        uint8_t warning_beeps;		// 1=A 2=B ... 0x1a=Z  0 = no alarm
        uint8_t sensor_text_id;		// constant value 0xd0
        uint8_t alarm_invers1;		// alarm bitmask. Value is displayed inverted
        uint8_t alarm_invers2;		// alarm bitmask. Value is displayed inverted                                                                
        uint8_t cell1;			// cell 1 voltage lower value. 0.02V steps, 124=2.48V
        uint8_t cell2;
        uint8_t cell3;
        uint8_t cell4;
        uint8_t cell5;
        uint8_t cell6;
        uint8_t batt1_L;		// battery 1 voltage LSB value. 0.1V steps. 50 = 5.5V
        uint8_t batt1_H;
        uint8_t batt2_L;		// battery 2 voltage LSB value. 0.1V steps. 50 = 5.5V
        uint8_t batt2_H;
        uint8_t temperature1;		// temperature 1. offset of 20. a value of 20 = 0°C
        uint8_t temperature2;		// temperature 2. offset of 20. a value of 20 = 0°C
        uint8_t fuel_procent;		// Fuel capacity in %. Values 0--100
					// graphical display ranges: 0-25% 50% 75% 100%
        uint8_t fuel_ml_L;		// Fuel in ml scale. Full = 65535!
        uint8_t fuel_ml_H;		//
        uint8_t rpm_L;			// RPM in 10 RPM steps. 300 = 3000rpm
        uint8_t rpm_H;			//
        uint8_t altitude_L;		// altitude in meters. offset of 500, 500 = 0m
        uint8_t altitude_H;		//
        uint8_t climbrate_L;		// climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
        uint8_t climbrate_H;		//
        uint8_t climbrate3s;		// climb rate in m/3sec. Value of 120 = 0m/3sec
        uint8_t current_L;		// current in 0.1A steps
        uint8_t current_H;		//
        uint8_t main_voltage_L;		// Main power voltage using 0.1V steps
        uint8_t main_voltage_H;		//
        uint8_t batt_cap_L;		// used battery capacity in 10mAh steps
        uint8_t batt_cap_H;		//
        uint8_t speed_L;		// (air?) speed in km/h(?) we are using ground speed here per default
        uint8_t speed_H;		//
        uint8_t min_cell_volt;		// minimum cell voltage in 2mV steps. 124 = 2,48V
        uint8_t min_cell_volt_num;	// number of the cell with the lowest voltage
        uint8_t rpm2_L;			// RPM in 10 RPM steps. 300 = 3000rpm
        uint8_t rpm2_H;			//
        uint8_t general_error_number;	// Voice error == 12. TODO: more docu
        uint8_t pressure;		// Pressure up to 16bar. 0,1bar scale. 20 = 2bar
        uint8_t version;		// version number TODO: more info?
        uint8_t stop;			// stop byte
        uint8_t checksum;		// checksum
};

/* GPS sensor constants. */
#define GPS_SENSOR_ID		0x8a
#define GPS_SENSOR_TEXT_ID	0xa0

/** 
 * The GPS sensor message
 * Struct based on: https://code.google.com/p/diy-hott-gps/downloads 
 */
struct gps_module_msg { 
	uint8_t start;			/**< Start byte */
	uint8_t sensor_id;		/**< GPS sensor ID*/
	uint8_t warning;		/**< Byte 3: 0…= warning beeps */
	uint8_t sensor_text_id;		/**< GPS Sensor text mode ID */
	uint8_t alarm_inverse1;		/**< Byte 5: 01 inverse status */
	uint8_t alarm_inverse2;		/**< Byte 6: 00 inverse status status 1 = no GPS Signal */
	uint8_t flight_direction;	/**< Byte 7: 119 = Flightdir./dir. 1 = 2°; 0° (North), 9 0° (East), 180° (South), 270° (West) */
	uint8_t gps_speed_L;		/**< Byte 8: 8 = /GPS speed low byte 8km/h */
	uint8_t gps_speed_H;		/**< Byte 9: 0 = /GPS speed high byte */
	  
	uint8_t latitude_ns;		/**< Byte 10: 000 = N = 48°39’988 */
	uint8_t latitude_min_L;		/**< Byte 11: 231 0xE7 = 0x12E7 = 4839 */
	uint8_t latitude_min_H;		/**< Byte 12: 018 18 = 0x12 */
	uint8_t latitude_sec_L;		/**< Byte 13: 171 220 = 0xDC = 0x03DC =0988 */
	uint8_t latitude_sec_H;		/**< Byte 14: 016 3 = 0x03 */
	 
	uint8_t longitude_ew;		/**< Byte 15: 000  = E= 9° 25’9360 */
	uint8_t longitude_min_L;	/**< Byte 16: 150 157 = 0x9D = 0x039D = 0925 */
	uint8_t longitude_min_H;	/**< Byte 17: 003 3 = 0x03 */
	uint8_t longitude_sec_L;	/**< Byte 18: 056 144 = 0x90 0x2490 = 9360*/
	uint8_t longitude_sec_H;	/**< Byte 19: 004 36 = 0x24 */
	  
	uint8_t distance_L;			/**< Byte 20: 027 123 = /distance low byte 6 = 6 m */
	uint8_t distance_H;			/**< Byte 21: 036 35 = /distance high byte */
	uint8_t altitude_L;			/**< Byte 22: 243 244 = /Altitude low byte 500 = 0m */
	uint8_t altitude_H;			/**< Byte 23: 001 1 = /Altitude high byte */
	uint8_t resolution_L;		/**< Byte 24: 48 = Low Byte m/s resolution 0.01m 48 = 30000 = 0.00m/s (1=0.01m/s) */
	uint8_t resolution_H;		/**< Byte 25: 117 = High Byte m/s resolution 0.01m */
	uint8_t unknown1;			/**< Byte 26: 120 = 0m/3s */
	uint8_t gps_num_sat;		/**< Byte 27: GPS.Satellites (number of satelites) (1 byte) */
	uint8_t gps_fix_char;		/**< Byte 28: GPS.FixChar. (GPS fix character. display, if DGPS, 2D oder 3D) (1 byte) */
	uint8_t home_direction;		/**< Byte 29: HomeDirection (direction from starting point to Model position) (1 byte) */
	uint8_t angle_x_direction;	/**< Byte 30: angle x-direction (1 byte) */
	uint8_t angle_y_direction;	/**< Byte 31: angle y-direction (1 byte) */
	uint8_t angle_z_direction;	/**< Byte 32: angle z-direction (1 byte) */
	uint8_t gyro_x_L;			/**< Byte 33: gyro x low byte (2 bytes) */
	uint8_t gyro_x_H;			/**< Byte 34: gyro x high byte */
	uint8_t gyro_y_L;			/**< Byte 35: gyro y low byte (2 bytes) */
	uint8_t gyro_y_H;			/**< Byte 36: gyro y high byte */
	uint8_t gyro_z_L;			/**< Byte 37: gyro z low byte (2 bytes) */
	uint8_t gyro_z_H;			/**< Byte 38: gyro z high byte */
	uint8_t vibration;			/**< Byte 39: vibration (1 bytes) */
	uint8_t ascii4;				/**< Byte 40: 00 ASCII Free Character [4] */
	uint8_t ascii5;				/**< Byte 41: 00 ASCII Free Character [5] */
	uint8_t gps_fix;			/**< Byte 42: 00 ASCII Free Character [6], we use it for GPS FIX */
	uint8_t version;			/**< Byte 43: 00 version number */
	uint8_t stop;				/**< Byte 44: 0x7D Ende byte */
	uint8_t checksum;			/**< Byte 45: Parity Byte */
};

// The maximum size of a message.
#define MAX_MESSAGE_BUFFER_SIZE 45

void init_sub_messages(void);
void init_pub_messages(void);
void build_gam_request(uint8_t *buffer, size_t *size);
void publish_gam_message(const uint8_t *buffer);
void build_eam_response(uint8_t *buffer, size_t *size);
void build_gam_response(uint8_t *buffer, size_t *size);
void build_gps_response(uint8_t *buffer, size_t *size);
float _get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);
void convert_to_degrees_minutes_seconds(double lat, int *deg, int *min, int *sec);


#endif /* MESSAGES_H_ */
