/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * @file frsky_data.c
 * @author Stefan Rado <px4@sradonia.net>
 *
 * FrSky telemetry implementation.
 *
 */

#include "frsky_data.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <arch/math.h>
#include <geo/geo.h>
#include <stdbool.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

/* FrSky sensor hub data IDs */
#define FRSKY_ID_GPS_ALT_BP     0x01
#define FRSKY_ID_TEMP1          0x02
#define FRSKY_ID_RPM            0x03
#define FRSKY_ID_FUEL           0x04
#define FRSKY_ID_TEMP2          0x05
#define FRSKY_ID_VOLTS          0x06
#define FRSKY_ID_GPS_ALT_AP     0x09
#define FRSKY_ID_BARO_ALT_BP    0x10
#define FRSKY_ID_GPS_SPEED_BP   0x11
#define FRSKY_ID_GPS_LONG_BP    0x12
#define FRSKY_ID_GPS_LAT_BP     0x13
#define FRSKY_ID_GPS_COURS_BP   0x14
#define FRSKY_ID_GPS_DAY_MONTH  0x15
#define FRSKY_ID_GPS_YEAR       0x16
#define FRSKY_ID_GPS_HOUR_MIN   0x17
#define FRSKY_ID_GPS_SEC        0x18
#define FRSKY_ID_GPS_SPEED_AP   0x19
#define FRSKY_ID_GPS_LONG_AP    0x1A
#define FRSKY_ID_GPS_LAT_AP     0x1B
#define FRSKY_ID_GPS_COURS_AP   0x1C
#define FRSKY_ID_BARO_ALT_AP    0x21
#define FRSKY_ID_GPS_LONG_EW    0x22
#define FRSKY_ID_GPS_LAT_NS     0x23
#define FRSKY_ID_ACCEL_X        0x24
#define FRSKY_ID_ACCEL_Y        0x25
#define FRSKY_ID_ACCEL_Z        0x26
#define FRSKY_ID_CURRENT        0x28
#define FRSKY_ID_VARIO          0x30
#define FRSKY_ID_VFAS           0x39
#define FRSKY_ID_VOLTS_BP       0x3A
#define FRSKY_ID_VOLTS_AP       0x3B

#define frac(f) (f - (int)f)

static struct battery_status_s *battery_status;
static struct vehicle_global_position_s *global_pos;
static struct sensor_combined_s *sensor_data;
static struct vehicle_status_s *vehicle_status;

static int battery_sub = -1;
static int global_position_sub = -1;
static int sensor_sub = -1;
static int vehicle_status_sub = -1;

/**
 * Initializes the uORB subscriptions.
 */
bool frsky_init()
{
	battery_status = malloc(sizeof(struct battery_status_s));
	global_pos = malloc(sizeof(struct vehicle_global_position_s));
	sensor_data = malloc(sizeof(struct sensor_combined_s));
	vehicle_status = malloc(sizeof(struct vehicle_status_s));

	if (battery_status == NULL || global_pos == NULL || sensor_data == NULL || vehicle_status == NULL) {
		return false;
	}

	battery_sub = orb_subscribe(ORB_ID(battery_status));
	global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	return true;
}

void frsky_deinit()
{
	free(battery_status);
	free(global_pos);
	free(sensor_data);
	free(vehicle_status);
}

/**
 * Sends a 0x5E start/stop byte.
 */
static void frsky_send_startstop(int uart)
{
	static const uint8_t c = 0x5E;
	write(uart, &c, sizeof(c));
}

/**
 * Sends one byte, performing byte-stuffing if necessary.
 */
static void frsky_send_byte(int uart, uint8_t value)
{
	const uint8_t x5E[] = { 0x5D, 0x3E };
	const uint8_t x5D[] = { 0x5D, 0x3D };

	switch (value) {
	case 0x5E:
		write(uart, x5E, sizeof(x5E));
		break;

	case 0x5D:
		write(uart, x5D, sizeof(x5D));
		break;

	default:
		write(uart, &value, sizeof(value));
		break;
	}
}

/**
 * Sends one data id/value pair.
 */
static void frsky_send_data(int uart, uint8_t id, int16_t data)
{
	/* Cast data to unsigned, because signed shift might behave incorrectly */
	uint16_t udata = data;

	frsky_send_startstop(uart);

	frsky_send_byte(uart, id);
	frsky_send_byte(uart, udata);      /* LSB */
	frsky_send_byte(uart, udata >> 8); /* MSB */
}

/**
 * Sends frame 1 (every 200ms):
 *   acceleration values, barometer altitude, temperature, battery voltage & current
 */
void frsky_send_frame1(int uart)
{
	/* get a local copy of the current sensor values */
	orb_copy(ORB_ID(sensor_combined), sensor_sub, sensor_data);

	/* get a local copy of the battery data */
	orb_copy(ORB_ID(battery_status), battery_sub, battery_status);

	/* send formatted frame */
	frsky_send_data(uart, FRSKY_ID_ACCEL_X,
			roundf(sensor_data->accelerometer_m_s2[0] * 1000.0f));
	frsky_send_data(uart, FRSKY_ID_ACCEL_Y,
			roundf(sensor_data->accelerometer_m_s2[1] * 1000.0f));
	frsky_send_data(uart, FRSKY_ID_ACCEL_Z,
			roundf(sensor_data->accelerometer_m_s2[2] * 1000.0f));

	frsky_send_data(uart, FRSKY_ID_BARO_ALT_BP,
			sensor_data->baro_alt_meter[0]);
	frsky_send_data(uart, FRSKY_ID_BARO_ALT_AP,
			roundf(frac(sensor_data->baro_alt_meter[0]) * 100.0f));

	frsky_send_data(uart, FRSKY_ID_TEMP1,
			roundf(sensor_data->baro_temp_celcius[0]));

	frsky_send_data(uart, FRSKY_ID_VFAS,
			roundf(battery_status->voltage_v * 10.0f));
	frsky_send_data(uart, FRSKY_ID_CURRENT,
			(battery_status->current_a < 0) ? 0 : roundf(battery_status->current_a * 10.0f));

	frsky_send_startstop(uart);
}

/**
 * Formats the decimal latitude/longitude to the required degrees/minutes.
 */
static float frsky_format_gps(float dec)
{
	float dm_deg = (int) dec;
	return (dm_deg * 100.0f) + (dec - dm_deg) * 60;
}

/**
 * Sends frame 2 (every 1000ms):
 *   GPS course, latitude, longitude, ground speed, GPS altitude, remaining battery level
 */
void frsky_send_frame2(int uart)
{
	/* get a local copy of the global position data */
	orb_copy(ORB_ID(vehicle_global_position), global_position_sub, global_pos);

	/* get a local copy of the vehicle status data */
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, vehicle_status);

	/* send formatted frame */
	float course = 0, lat = 0, lon = 0, speed = 0, alt = 0;
	char lat_ns = 0, lon_ew = 0;
	int sec = 0;

	if (global_pos->timestamp != 0 && hrt_absolute_time() < global_pos->timestamp + 20000) {
		time_t time_gps = global_pos->time_utc_usec / 1000000ULL;
		struct tm *tm_gps = gmtime(&time_gps);

		course = (global_pos->yaw + M_PI_F) / M_PI_F * 180.0f;
		lat    = frsky_format_gps(fabsf(global_pos->lat));
		lat_ns = (global_pos->lat < 0) ? 'S' : 'N';
		lon    = frsky_format_gps(fabsf(global_pos->lon));
		lon_ew = (global_pos->lon < 0) ? 'W' : 'E';
		speed  = sqrtf(global_pos->vel_n * global_pos->vel_n + global_pos->vel_e * global_pos->vel_e)
			 * 25.0f / 46.0f;
		alt    = global_pos->alt;
		sec    = tm_gps->tm_sec;
	}

	frsky_send_data(uart, FRSKY_ID_GPS_COURS_BP, course);
	frsky_send_data(uart, FRSKY_ID_GPS_COURS_AP, frac(course) * 1000.0f);

	frsky_send_data(uart, FRSKY_ID_GPS_LAT_BP, lat);
	frsky_send_data(uart, FRSKY_ID_GPS_LAT_AP, frac(lat) * 10000.0f);
	frsky_send_data(uart, FRSKY_ID_GPS_LAT_NS, lat_ns);

	frsky_send_data(uart, FRSKY_ID_GPS_LONG_BP, lon);
	frsky_send_data(uart, FRSKY_ID_GPS_LONG_AP, frac(lon) * 10000.0f);
	frsky_send_data(uart, FRSKY_ID_GPS_LONG_EW, lon_ew);

	frsky_send_data(uart, FRSKY_ID_GPS_SPEED_BP, speed);
	frsky_send_data(uart, FRSKY_ID_GPS_SPEED_AP, frac(speed) * 100.0f);

	frsky_send_data(uart, FRSKY_ID_GPS_ALT_BP, alt);
	frsky_send_data(uart, FRSKY_ID_GPS_ALT_AP, frac(alt) * 100.0f);

	frsky_send_data(uart, FRSKY_ID_FUEL,
			roundf(vehicle_status->battery_remaining * 100.0f));

	frsky_send_data(uart, FRSKY_ID_GPS_SEC, sec);

	frsky_send_startstop(uart);
}

/**
 * Sends frame 3 (every 5000ms):
 *   GPS date & time
 */
void frsky_send_frame3(int uart)
{
	/* get a local copy of the battery data */
	orb_copy(ORB_ID(vehicle_global_position), global_position_sub, global_pos);

	/* send formatted frame */
	time_t time_gps = global_pos->time_utc_usec / 1000000ULL;
	struct tm *tm_gps = gmtime(&time_gps);
	uint16_t hour_min = (tm_gps->tm_min << 8) | (tm_gps->tm_hour & 0xff);
	frsky_send_data(uart, FRSKY_ID_GPS_DAY_MONTH, tm_gps->tm_mday);
	frsky_send_data(uart, FRSKY_ID_GPS_YEAR, tm_gps->tm_year);
	frsky_send_data(uart, FRSKY_ID_GPS_HOUR_MIN, hour_min);
	frsky_send_data(uart, FRSKY_ID_GPS_SEC, tm_gps->tm_sec);

	frsky_send_startstop(uart);
}

/* parse 11 byte frames */
bool frsky_parse_host(uint8_t *sbuf, int nbytes, struct adc_linkquality *v)
{
	bool data_ready = false;
	static int dcount = 0;
	static uint8_t type = 0;
	static uint8_t data[11];
	static enum {
		HEADER = 0,
		TYPE,
		DATA,
		TRAILER
	} state = HEADER;

	for (int i = 0; i < nbytes; i++) {
		switch (state) {
		case HEADER:
			if (sbuf[i] == 0x7E) {
				state = TYPE;
			}

			break;

		case TYPE:
			if (sbuf[i] != 0x7E) {
				state = DATA;
				type = sbuf[i];
				dcount = 0;
			}

			break;

		case DATA:

			/* read 8 data bytes */
			if (dcount < 7) {
				data[dcount++] = sbuf[i];

			} else {
				/* received all data bytes */
				state = TRAILER;
			}

			break;

		case TRAILER:
			state = HEADER;

			if (sbuf[i] != 0x7E) {
//				warnx("host packet error: %x", sbuf[i]);

			} else {
				data_ready = true;

				if (type == 0xFE) {
					/* this is an adc_linkquality packet */
					v->ad1 = data[0];
					v->ad2 = data[1];
					v->linkq = data[2];
				}
			}

			break;
		}
	}

	return data_ready;
}

