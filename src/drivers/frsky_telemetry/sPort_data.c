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
 * @file sPort_data.c
 * @author Stefan Rado <px4@sradonia.net>
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky SmartPort telemetry implementation.
 *
 */

#include "sPort_data.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>


#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

static int sensor_sub = -1;
static int global_position_sub = -1;
static int local_position_sub = -1;
static int battery_status_sub = -1;
static int vehicle_status_sub = -1;
static int gps_position_sub = -1;

static struct sensor_combined_s *sensor_combined;
static struct vehicle_global_position_s *global_pos;
static struct vehicle_local_position_s *local_pos;
static struct battery_status_s *battery_status;
static struct vehicle_status_s *vehicle_status;
static struct vehicle_gps_position_s *gps_position;


/**
 * Initializes the uORB subscriptions.
 */
bool sPort_init()
{

	sensor_combined = malloc(sizeof(struct sensor_combined_s));
	global_pos = malloc(sizeof(struct vehicle_global_position_s));
	local_pos = malloc(sizeof(struct vehicle_local_position_s));
	battery_status = malloc(sizeof(struct battery_status_s));
	vehicle_status = malloc(sizeof(struct vehicle_status_s));
	gps_position = malloc(sizeof(struct vehicle_gps_position_s));


	if (sensor_combined == NULL || global_pos == NULL || local_pos == NULL || battery_status == NULL
	    || vehicle_status == NULL || gps_position == NULL) {
		return false;
	}


	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	return true;
}

void sPort_deinit()
{
	free(sensor_combined);
	free(global_pos);
	free(local_pos);
	free(battery_status);
	free(vehicle_status);
	free(gps_position);
}

void sPort_update_topics()
{
	bool updated;
	/* get a local copy of the current sensor values */
	orb_check(sensor_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_combined), sensor_sub, sensor_combined);
	}

	/* get a local copy of the battery data */
	orb_check(battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), battery_status_sub, battery_status);
	}

	/* get a local copy of the global position data */
	orb_check(global_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_global_position), global_position_sub, global_pos);
	}

	/* get a local copy of the local position data */
	orb_check(local_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), local_position_sub, local_pos);
	}

	/* get a local copy of the vehicle status data */
	orb_check(vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, vehicle_status);
	}

	/* get a local copy of the gps position data */
	orb_check(gps_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_gps_position), gps_position_sub, gps_position);
	}

}


static void update_crc(uint16_t *crc, unsigned char b)
{
	*crc += b;
	*crc += *crc >> 8;
	*crc &= 0xFF;
}

/**
 * Sends one byte, performing byte-stuffing if necessary.
 */
static void sPort_send_byte(int uart, uint8_t value)
{
	const uint8_t x7E[] = { 0x7D, 0x5E };
	const uint8_t x7D[] = { 0x7D, 0x5D };

	switch (value) {
	case 0x7E:
		write(uart, x7E, sizeof(x7E));
		break;

	case 0x7D:
		write(uart, x7D, sizeof(x7D));
		break;

	default:
		write(uart, &value, sizeof(value));
		break;
	}
}

/**
 * Sends one data id/value pair.
 */
void sPort_send_data(int uart, uint16_t id, uint32_t data)
{
	union {
		uint32_t word;
		uint8_t byte[4];
	} buf;

	/* send start byte */
	static const uint8_t c = 0x10;
	write(uart, &c, 1);

	/* init crc */
	uint16_t crc = c;

	buf.word = id;

	for (int i = 0; i < 2; i++) {
		update_crc(&crc, buf.byte[i]);
		sPort_send_byte(uart, buf.byte[i]);      /* LSB first */
	}

	buf.word = data;

	for (int i = 0; i < 4; i++) {
		update_crc(&crc, buf.byte[i]);
		sPort_send_byte(uart, buf.byte[i]);      /* LSB first */
	}

	sPort_send_byte(uart, 0xFF - crc);
}


// scaling correct with OpenTX 2.1.7
void sPort_send_BATV(int uart)
{
	/* send battery voltage as VFAS */
	uint32_t voltage = (int)(100 * battery_status->voltage_v);
	sPort_send_data(uart, SMARTPORT_ID_VFAS, voltage);
}

// verified scaling
void sPort_send_CUR(int uart)
{
	/* send data */
	uint32_t current = (int)(10 * battery_status->current_a);
	sPort_send_data(uart, SMARTPORT_ID_CURR, current);
}

// verified scaling for "custom" altitude option
// OpenTX uses the initial reading as field elevation and displays
// the difference (altitude - field)
void sPort_send_ALT(int uart)
{
	/* send data */
	uint32_t alt = (int)(100 * sensor_combined->baro_alt_meter);
	sPort_send_data(uart, SMARTPORT_ID_ALT, alt);
}

// verified scaling for "calculated" option
void sPort_send_SPD(int uart)
{
	/* send data for A2 */
	float speed  = sqrtf(global_pos->vel_n * global_pos->vel_n + global_pos->vel_e * global_pos->vel_e);
	uint32_t ispeed = (int)(10 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

// TODO: verify scaling
void sPort_send_VSPD(int uart, float speed)
{
	/* send data for VARIO vertical speed: int16 cm/sec */
	int32_t ispeed = (int)(100 * speed);
	sPort_send_data(uart, SMARTPORT_ID_VARIO, ispeed);
}

// verified scaling
void sPort_send_FUEL(int uart)
{
	/* send data */
	uint32_t fuel = (int)(100 * battery_status->remaining);
	sPort_send_data(uart, SMARTPORT_ID_FUEL, fuel);
}

void sPort_send_GPS_LON(int uart)
{
	/* send longitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	/* precision is approximately 0.1m */
	uint32_t iLon =  6E-2 * fabs(gps_position->lon);

	iLon |= (1 << 31);

	if (gps_position->lon < 0) { iLon |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLon);
}

void sPort_send_GPS_LAT(int uart)
{
	/* send latitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 0 and bit 30=sign */
	uint32_t iLat = 6E-2 * fabs(gps_position->lat);

	if (gps_position->lat < 0) { iLat |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLat);
}

void sPort_send_GPS_ALT(int uart)
{
	/* send altitude */
	uint32_t iAlt = gps_position->alt / 10;
	sPort_send_data(uart, SMARTPORT_ID_GPS_ALT, iAlt);
}

void sPort_send_GPS_CRS(int uart)
{
	/* send course */

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	int32_t iYaw = local_pos->yaw * 18000.0f / M_PI_F;

	if (iYaw < 0) { iYaw += 36000; }

	sPort_send_data(uart, SMARTPORT_ID_GPS_CRS, iYaw);
}

void sPort_send_GPS_TIME(int uart)
{
	static int date = 0;

	/* send formatted frame */
	time_t time_gps = gps_position->time_utc_usec / 1000000ULL;
	struct tm *tm_gps = gmtime(&time_gps);

	if (date) {

		sPort_send_data(uart, SMARTPORT_ID_GPS_TIME,
				(uint32_t) 0xff | (tm_gps->tm_mday << 8) | ((tm_gps->tm_mon + 1) << 16) | ((tm_gps->tm_year - 100) << 24));
		date = 0;

	} else {

		sPort_send_data(uart, SMARTPORT_ID_GPS_TIME,
				(uint32_t) 0x00 | (tm_gps->tm_sec << 8) | (tm_gps->tm_min  << 16) | (tm_gps->tm_hour << 24));
		date = 1;

	}
}

void sPort_send_GPS_SPD(int uart)
{
	/* send 100 * knots */
	float speed  = sqrtf(global_pos->vel_n * global_pos->vel_n + global_pos->vel_e * global_pos->vel_e);
	uint32_t ispeed = (int)(1944 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

/*
 * Sends nav_state + 128
 */
void sPort_send_NAV_STATE(int uart)
{
	uint32_t navstate = (int)(128 + vehicle_status->nav_state);

	/* send data */
	sPort_send_data(uart, SMARTPORT_ID_DIY_NAVSTATE, navstate);
}

// verified scaling
// sends number of sats and type of gps fix
void sPort_send_GPS_FIX(int uart)
{
	/* send data */
	uint32_t satcount = (int)(gps_position->satellites_used);
	uint32_t fixtype = (int)(gps_position->fix_type);
	uint32_t t2 = satcount * 10 + fixtype;
	sPort_send_data(uart, SMARTPORT_ID_DIY_GPSFIX, t2);
}
