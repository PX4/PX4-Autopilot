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
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

static int battery_sub = -1;
static int sensor_sub = -1;
static int global_position_sub = -1;
static int vehicle_status_sub = -1;

/**
 * Initializes the uORB subscriptions.
 */
void sPort_init()
{
	battery_sub = orb_subscribe(ORB_ID(battery_status));
	global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
}

/**
 * Sends a 0x10 start byte.
 */
//static void sPort_send_start(int uart)
//{
//	static const uint8_t c = 0x10;
//	write(uart, &c, 1);
//}

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


// TODO: correct scaling
void sPort_send_BATV(int uart)
{
	/* get a local copy of the vehicle status data */
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);

	/* send battery voltage as VFAS */
	uint32_t voltage = (int)(255 * vehicle_status.battery_voltage / 16.8f);
	sPort_send_data(uart, SMARTPORT_ID_VFAS, voltage);
}

// TODO: correct scaling
void sPort_send_CUR(int uart)
{
	/* get a local copy of the vehicle status data */
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);

	/* send data */
	uint32_t current = (int)(255 * vehicle_status.battery_current / 50.0f);
	sPort_send_data(uart, SMARTPORT_ID_CURR, current);
}

// TODO: verify scaling
void sPort_send_ALT(int uart)
{
	/* get a local copy of the current sensor values */
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

	/* send data */
	uint32_t alt = (int)(raw.baro_alt_meter[0]);
	sPort_send_data(uart, SMARTPORT_ID_ALT, alt);
}

// TODO: verify scaling
void sPort_send_SPD(int uart)
{
	/* get a local copy of the global position data */
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_pos);

	/* send data for A2 */
	float speed  = sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e);
	uint32_t ispeed = (int)speed;
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

// TODO: verify scaling
void sPort_send_FUEL(int uart)
{
	/* get a local copy of the vehicle status data */
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);

	/* send data */
	uint32_t fuel = (int)(100 * vehicle_status.battery_remaining);
	sPort_send_data(uart, SMARTPORT_ID_FUEL, fuel);
}
