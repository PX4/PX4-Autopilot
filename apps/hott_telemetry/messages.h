/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Simon Wilks <sjwilks@gmail.com>
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
 *
 * Graupner HoTT Telemetry message generation.
 *
 */
#ifndef MESSAGES_H_
#define MESSAGES_H

#include <stdlib.h>

/* The buffer size used to store messages. This must be at least as big as the number of
 * fields in the largest message struct.
 */
#define MESSAGE_BUFFER_SIZE 50

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

/* Electric Air Module (EAM) constants. */
#define ELECTRIC_AIR_MODULE	0x8e
#define EAM_SENSOR_ID		0xe0

/* The Electric Air Module message. */
struct eam_module_msg {
	uint8_t start;			/**< Start byte   				*/
	uint8_t eam_sensor_id;		/**< EAM sensor ID 				*/
	uint8_t warning;
	uint8_t sensor_id;		/**< Sensor ID, why different? 			*/
	uint8_t alarm_inverse1;
	uint8_t alarm_inverse2;
	uint8_t cell1_L;		/**< Lipo cell voltages. Not supported. 	*/
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
	uint8_t temperature1;		/**< Temperature sensor 1. 20 = 0 degrees 		*/
	uint8_t temperature2;
	uint8_t altitude_L;		/**< Attitude (meters) lower 8-bits. 500 = 0 meters 	*/
	uint8_t altitude_H;
	uint8_t current_L;		/**< Current (mAh) lower 8-bits in steps of 0.1V 	*/
	uint8_t current_H;
	uint8_t main_voltage_L;		/**< Main power voltage lower 8-bits in steps of 0.1V 	*/
	uint8_t main_voltage_H;
	uint8_t battery_capacity_L;	/**< Used battery capacity in steps of 10mAh 		*/
	uint8_t battery_capacity_H;
	uint8_t climbrate_L;		/**< Climb rate in 0.01m/s. 0m/s = 30000 		*/
	uint8_t climbrate_H;
	uint8_t climbrate_3s;		/**< Climb rate in m/3sec. 0m/3sec = 120 		*/
	uint8_t rpm_L;			/**< RPM Lower 8-bits In steps of 10 U/min 		*/
	uint8_t rpm_H;
	uint8_t electric_min;		/**< Flight time in minutes. 				*/
	uint8_t electric_sec;		/**< Flight time in seconds. 				*/
	uint8_t speed_L;		/**< Airspeed in km/h in steps of 1 km/h 		*/
	uint8_t speed_H;
	uint8_t stop;			/**< Stop byte 						*/
	uint8_t checksum;		/**< Lower 8-bits of all bytes summed. 			*/
};

void messages_init(void);
void build_eam_response(uint8_t *buffer, int *size);

#endif /* MESSAGES_H_ */
