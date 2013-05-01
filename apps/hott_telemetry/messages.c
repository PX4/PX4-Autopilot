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
 * @file messages.c
 *
 */

#include "messages.h"

#include <string.h>
#include <systemlib/systemlib.h>
#include <unistd.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

/* The board is very roughly 5 deg warmer than the surrounding air */
#define BOARD_TEMP_OFFSET_DEG 5

static int battery_sub = -1;
static int gps_sub = -1;
static int sensor_sub = -1;

void 
messages_init(void)
{
	battery_sub = orb_subscribe(ORB_ID(battery_status));
	gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
}

void 
build_eam_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the current sensor values */
	struct sensor_combined_s raw = { 0 };
	orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

	/* get a local copy of the battery data */
	struct battery_status_s battery = { 0 };
	orb_copy(ORB_ID(battery_status), battery_sub, &battery);

	struct eam_module_msg msg = { 0 };
	*size = sizeof(msg);

	msg.start = START_BYTE;
	msg.eam_sensor_id = EAM_SENSOR_ID;
	msg.sensor_id = EAM_SENSOR_TEXT_ID;
	
	msg.temperature1 = (uint8_t)(raw.baro_temp_celcius + 20);
	msg.temperature2 = msg.temperature1 - BOARD_TEMP_OFFSET_DEG;

	msg.main_voltage_L = (uint8_t)(battery.voltage_v * 10);

	uint16_t alt = (uint16_t)(raw.baro_alt_meter + 500);
	msg.altitude_L = (uint8_t)alt & 0xff;
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

	// TODO: flight time
	// TODO: climb rate
	

	msg.stop = STOP_BYTE;

	memcpy(buffer, &msg, *size);
}

void 
build_gps_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the current sensor values */
	struct sensor_combined_s raw = { 0 };
	orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

 	/* get a local copy of the battery data */
	struct vehicle_gps_position_s gps = { 0 };
	orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps);

	struct gps_module_msg msg = { 0 };
	*size = sizeof(msg);

	msg.start = START_BYTE;
	msg.sensor_id = GPS_SENSOR_ID;
	msg.sensor_text_id = GPS_SENSOR_TEXT_ID;

	uint16_t speed = (uint16_t)(gps.vel_m_s * 3.6);
	msg.gps_speed_L = (uint8_t)speed & 0xff;
	msg.gps_speed_H = (uint8_t)(speed >> 8) & 0xff;
	
	uint16_t alt = (uint16_t)(gps.alt);
	msg.altitude_L = (uint8_t)alt & 0xff;
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

	msg.gps_num_sat = gps.satellites_visible;

	/* Display the fix type: 0 = none, 2 = 2D, 3 = 3D */
	msg.gps_fix = (uint8_t)(gps.fix_type + 48);

	// TODO(sjwilks): Complete.

	msg.stop = STOP_BYTE;

	memcpy(buffer, &msg, *size);
}
