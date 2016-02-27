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

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

static int sensor_sub = -1;
static int global_position_sub = -1;
static int vehicle_status_sub = -1;
static struct vehicle_status_s *vehicle_status;
static struct sensor_combined_s *sensor_data;
static struct vehicle_global_position_s *global_pos;

/**
 * Initializes the uORB subscriptions.
 */
bool sPort_init()
{
	vehicle_status = malloc(sizeof(struct vehicle_status_s));
	sensor_data = malloc(sizeof(struct sensor_combined_s));
	global_pos = malloc(sizeof(struct vehicle_global_position_s));

	if (vehicle_status == NULL || sensor_data == NULL || global_pos == NULL) {
		return false;
	}

	global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	return true;
}

void sPort_deinit()
{
	free(vehicle_status);
	free(sensor_data);
	free(global_pos);
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
	/* get a local copy of the vehicle status data */
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, vehicle_status);

	/* send battery voltage as VFAS */
	uint32_t voltage = (int)(100 * vehicle_status->battery_voltage);
	sPort_send_data(uart, SMARTPORT_ID_VFAS, voltage);
}

// verified scaling
void sPort_send_CUR(int uart)
{
	/* send data */
	uint32_t current = (int)(10 * vehicle_status->battery_current);
	sPort_send_data(uart, SMARTPORT_ID_CURR, current);
}

// verified scaling for "custom" altitude option
// OpenTX uses the initial reading as field elevation and displays
// the difference (altitude - field)
void sPort_send_ALT(int uart)
{
	/* get a local copy of the current sensor values */
	orb_copy(ORB_ID(sensor_combined), sensor_sub, sensor_data);

	/* send data */
	uint32_t alt = (int)(100 * sensor_data->baro_alt_meter[0]);
	sPort_send_data(uart, SMARTPORT_ID_ALT, alt);
}

// verified scaling for "calculated" option
void sPort_send_SPD(int uart)
{
	/* send data */
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
	uint32_t fuel = (int)(100 * vehicle_status->battery_remaining);
	sPort_send_data(uart, SMARTPORT_ID_FUEL, fuel);
}

void sPort_send_GPS_LON(int uart)
{
	/* send longitude */
	/* get a local copy of the global position data */
	orb_copy(ORB_ID(vehicle_global_position), global_position_sub, global_pos);

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	/* precision is approximately 0.1m */
	uint32_t iLon = 6E5 * fabs(global_pos->lon);
	iLon |= (1 << 31);

	if (global_pos->lon < 0) { iLon |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLon);
}

void sPort_send_GPS_LAT(int uart)
{
	/* send latitude */

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 0 and bit 30=sign */
	uint32_t iLat = 6E5 * fabs(global_pos->lat);

	if (global_pos->lat < 0) { iLat |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLat);
}

void sPort_send_GPS_ALT(int uart)
{
	/* send altitude */

	/* convert to 100 * m/sec */
	uint32_t iAlt = 100 * global_pos->alt;
	sPort_send_data(uart, SMARTPORT_ID_GPS_ALT, iAlt);
}

void sPort_send_GPS_COG(int uart)
{
	/* send course */

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	uint32_t iYaw = 100 * global_pos->yaw;
	sPort_send_data(uart, SMARTPORT_ID_GPS_CRS, iYaw);
}

void sPort_send_GPS_SPD(int uart)
{
	/* send 100 * knots */
	float speed  = sqrtf(global_pos->vel_n * global_pos->vel_n + global_pos->vel_e * global_pos->vel_e);
	uint32_t ispeed = (int)(1944 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}
