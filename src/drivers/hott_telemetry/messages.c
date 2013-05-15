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

#include <math.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <unistd.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

/* The board is very roughly 5 deg warmer than the surrounding air */
#define BOARD_TEMP_OFFSET_DEG 5

static int battery_sub = -1;
static int gps_sub = -1;
static int home_sub = -1;
static int sensor_sub = -1;

void 
messages_init(void)
{
	battery_sub = orb_subscribe(ORB_ID(battery_status));
	gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	home_sub = orb_subscribe(ORB_ID(home_position));
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

	/* Current flight direction */
	msg.flight_direction = (uint8_t)(gps.cog_rad * M_RAD_TO_DEG_F);

	/* GPS speed */
	uint16_t speed = (uint16_t)(gps.vel_m_s * 3.6);
	msg.gps_speed_L = (uint8_t)speed & 0xff;
	msg.gps_speed_H = (uint8_t)(speed >> 8) & 0xff;
	
	/* Get latitude in degrees, minutes and seconds */
	double lat = ((double)(gps.lat)) * 1e-7d;

	msg.latitude_ns = 0;
	if (lat < 0) {
		msg.latitude_ns = 1;
		lat = -lat;
	}

	int deg;
	int min;
	int sec;
	convert_to_degrees_minutes_seconds(lat, &deg, &min, &sec);

	uint16_t lat_min = (uint16_t)(deg * 100 + min);
	msg.latitude_min_L = (uint8_t)lat_min & 0xff;
	msg.latitude_min_H = (uint8_t)(lat_min >> 8) & 0xff;
	uint16_t lat_sec = (uint16_t)(sec);
	msg.latitude_sec_L = (uint8_t)lat_sec & 0xff;
	msg.latitude_sec_H = (uint8_t)(lat_sec >> 8) & 0xff;


	/* Get longitude in degrees, minutes and seconds */
	double lon = ((double)(gps.lon)) * 1e-7d;

	msg.longitude_ew = 0;
	if (lon < 0) {
		msg.longitude_ew = 1;
		lon = -lon;
	}

	convert_to_degrees_minutes_seconds(lon, &deg, &min, &sec);
	
	uint16_t lon_min = (uint16_t)(deg * 100 + min);
	msg.longitude_min_L = (uint8_t)lon_min & 0xff;
	msg.longitude_min_H = (uint8_t)(lon_min >> 8) & 0xff;
	uint16_t lon_sec = (uint16_t)(sec);
	msg.longitude_sec_L = (uint8_t)lon_sec & 0xff;
	msg.longitude_sec_H = (uint8_t)(lon_sec >> 8) & 0xff;
	
	/* Altitude */
	uint16_t alt = (uint16_t)(gps.alt * 1e-3 + 500.0f);
	msg.altitude_L = (uint8_t)alt & 0xff;
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

	/* Distance from home */
	bool updated;
	orb_check(home_sub, &updated);
	if (updated) {
	    /* get a local copy of the home position data */
		struct home_position_s home = { 0 };
		orb_copy(ORB_ID(home_position), home_sub, &home);

	    uint16_t dist = (uint16_t)get_distance_to_next_waypoint(
	    	(double)home.lat*1e-7, (double)home.lon*1e-7, lat, lon);
	    warnx("dist %d home.lat %3.6f home.lon %3.6f lat %3.6f lon %3.6f ", 
	    	  dist, (double)home.lat*1e-7d, (double)home.lon*1e-7d, lat, lon);
		msg.distance_L = (uint8_t)dist & 0xff;
		msg.distance_H = (uint8_t)(dist >> 8) & 0xff;

		/* Direction back to home */
		uint16_t bearing = (uint16_t)get_bearing_to_next_waypoint(
	    	(double)home.lat*1e-7, (double)home.lon*1e-7, lat, lon) * M_RAD_TO_DEG_F;
		msg.home_direction = (uint8_t)bearing >> 1;
    }

	msg.gps_num_sat = gps.satellites_visible;

	/* The GPS fix type: 0 = none, 2 = 2D, 3 = 3D */
	msg.gps_fix_char = (uint8_t)(gps.fix_type + 48);
	msg.gps_fix = (uint8_t)(gps.fix_type + 48);

	msg.stop = STOP_BYTE;

	memcpy(buffer, &msg, *size);
}

void
convert_to_degrees_minutes_seconds(double val, int *deg, int *min, int *sec)
{
    *deg = (int)val;

    double delta = val - *deg;
    *min = (int)(delta * 60.0);
    *sec = (int)(delta * 3600.0);
}
