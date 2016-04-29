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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/mavlink_log.h>

#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

static int sensor_sub = -1;
static int global_position_sub = -1;
static int battery_status_sub = -1;
static int vehicle_status_sub = -1;
static int gps_position_sub = -1;
static int vehicle_attitude_sub = -1;
static int mission_result_sub = -1;
static int mavlink_log_sub = -1;

static struct sensor_combined_s *sensor_combined;
static struct vehicle_global_position_s *global_pos;
static struct battery_status_s *battery_status;
static struct vehicle_status_s *vehicle_status;
static struct vehicle_gps_position_s *gps_position;
static struct vehicle_attitude_s *vehicle_attitude;
static struct mission_result_s *mission_result;
static struct mavlink_log_s *mavlink_log;

//FIFO for mavlink messages
#define FIFO_ELEMENTS 636 //12*52, so up to 12 messages, worst case.
#define FIFO_SIZE (FIFO_ELEMENTS + 1)
uint8_t fifo[FIFO_SIZE];
int fifoIn, fifoOut;

/**
 * Initializes the uORB subscriptions.
 */
bool sPort_init()
{

	sensor_combined = malloc(sizeof(struct sensor_combined_s));
	global_pos = malloc(sizeof(struct vehicle_global_position_s));
	battery_status = malloc(sizeof(struct battery_status_s));
	vehicle_status = malloc(sizeof(struct vehicle_status_s));
	gps_position = malloc(sizeof(struct vehicle_gps_position_s));
	vehicle_attitude = malloc(sizeof(struct vehicle_attitude_s));
	mission_result = malloc(sizeof(struct mission_result_s));
	mavlink_log = malloc(sizeof(struct mavlink_log_s));

	if (sensor_combined == NULL || global_pos == NULL || battery_status == NULL || vehicle_status == NULL
	    || gps_position == NULL || vehicle_attitude == NULL || mission_result == NULL || mavlink_log == NULL) {
		return false;
	}


	sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	mission_result_sub = orb_subscribe(ORB_ID(mission_result));
	mavlink_log_sub = orb_subscribe(ORB_ID(mavlink_log));

	fifo_init();

	return true;
}

void sPort_deinit()
{
	free(sensor_combined);
	free(global_pos);
	free(battery_status);
	free(vehicle_status);
	free(gps_position);
	free(vehicle_attitude);
	free(mission_result);
	free(mavlink_log);
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

	/* get a local copy of the vehicle attitude data */
	orb_check(vehicle_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, vehicle_attitude);
	}

	/* get a local copy of the mission result data */
	orb_check(mission_result_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mission_result), mission_result_sub, mission_result);
	}

	/* get a local copy of the mavlink log data */
	orb_check(mavlink_log_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mavlink_log), mavlink_log_sub, mavlink_log);

		uint8_t startbyte = 0x02;
		uint8_t endbyte = 0x03;

		if (mavlink_log->severity <= 6) {

			fifo_push(startbyte); //start byte

			for (int x = 0; x <= 50; x++) {

				if (mavlink_log->text[x] == '\0') { break; };

				if (fifo_push(mavlink_log->text[x])) {
					warnx("mavlink byte fifo is full!");
					uint8_t last_element;
					fifo_pop(&last_element);
					fifo_push(endbyte); //end byte
					return;
				}
			}

			fifo_push(endbyte); //end byte
		}
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
void sPort_send_VFAS(int uart)
{
	/* send battery voltage as VFAS */
	uint32_t voltage = (int)(100 * battery_status->voltage_v);
	sPort_send_data(uart, SMARTPORT_ID_VFAS, voltage);
}

void sPort_send_CELLS(int uart)
{
	/* send battery voltage as VFAS */
	uint32_t voltage_cells = (int)((100 * battery_status->voltage_v) / battery_status->cell_count);
	sPort_send_data(uart, SMARTPORT_ID_CELLS, voltage_cells);
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
	uint32_t alt = (int)(100 * sensor_combined->baro_alt_meter[0]);
	sPort_send_data(uart, SMARTPORT_ID_ALT, alt);
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

void sPort_send_ACCX(int uart)
{
	/* send data. opentx expects acc values in g. */
	sPort_send_data(uart, SMARTPORT_ID_ACCX, roundf(sensor_combined->accelerometer_m_s2[0] * 1000.0f * (1/9.81f)));
}

void sPort_send_ACCY(int uart)
{
	/* send data. opentx expects acc values in g. */
	sPort_send_data(uart, SMARTPORT_ID_ACCY, roundf(sensor_combined->accelerometer_m_s2[1] * 1000.0f * (1/9.81f)));
}

void sPort_send_ACCZ(int uart)
{
	/* send data. opentx expects acc values in g. */
	sPort_send_data(uart, SMARTPORT_ID_ACCZ, roundf(sensor_combined->accelerometer_m_s2[2] * 1000.0f * (1/9.81f)));
}

void sPort_send_GPS_LON(int uart)
{
	/* send longitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	/* precision is approximately 0.1m */
	uint32_t iLon = 6E5 * fabs(gps_position->lon);
	iLon |= (1 << 31);

	if (gps_position->lon < 0) { iLon |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLon);
}

void sPort_send_GPS_LAT(int uart)
{
	/* send latitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 0 and bit 30=sign */
	uint32_t iLat = 6E5 * fabs(gps_position->lat);

	if (gps_position->lat < 0) { iLat |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLat);
}

void sPort_send_GPS_ALT(int uart)
{
	/* send altitude */
	/* convert to 100 * m/sec */
	uint32_t iAlt = 100 * gps_position->alt;
	sPort_send_data(uart, SMARTPORT_ID_GPS_ALT, iAlt);
}

void sPort_send_GPS_CRS(int uart)
{
	/* send course */

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	uint32_t iYaw = 100 * global_pos->yaw;
	sPort_send_data(uart, SMARTPORT_ID_GPS_CRS, iYaw);
}

/*
	OpenTX Format in binary:
	if last 4 bit are 0000:
		HHHHMMMMSSSS0001 (H = Hour, M = Minutes, S = Seconds)
	else
		YYYYMMMMDDDD0000 (Y = Year, M = Month, D = Day)

	see https://github.com/opentx/opentx/blob/master/radio/src/telemetry/telemetry.cpp
*/

void sPort_send_GPS_DATE(int uart)
{

	time_t time_gps = gps_position->time_utc_usec / 1000000ULL; //1000000ULL = Number of microseconds in milliseconds
	struct tm *tm_gps = gmtime(&time_gps);
	uint8_t year = tm_gps->tm_year; //years since 1900
	uint8_t month = tm_gps->tm_mon; //0-11
	uint8_t day = tm_gps->tm_mday; //1-31

	uint32_t frsky_time_ymd = (year << 24) | month << 16 | day << 8 | 1;

	sPort_send_data(uart, SMARTPORT_ID_GPS_TIME, frsky_time_ymd);
}

void sPort_send_GPS_TIME(int uart)
{
	time_t time_gps = gps_position->time_utc_usec / 1000000ULL; //1000000ULL = Number of microseconds in milliseconds
	struct tm *tm_gps = gmtime(&time_gps);

	uint8_t hour = tm_gps->tm_hour; //0-23
	uint8_t minute = tm_gps->tm_min; //0-59
	uint8_t second = tm_gps->tm_sec; //0-60

	uint32_t frsky_time_hms = (hour << 24) | minute << 16 | second << 8 | 0;

	sPort_send_data(uart, SMARTPORT_ID_GPS_TIME, frsky_time_hms);
}

void sPort_send_GPS_SPD(int uart)
{
	/* send 100 * knots */
	float speed  = sqrtf(global_pos->vel_n * global_pos->vel_n + global_pos->vel_e * global_pos->vel_e);
	uint32_t ispeed = (int)(1944 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

// verified scaling
// sends number of sats and type of gps fix
void sPort_send_GPS_FIX(int uart)
{
	/* send data */
	uint32_t satcount = (int)(gps_position->satellites_used);
	uint32_t fixtype = (int)(gps_position->fix_type);
	uint32_t t2 = satcount * 10 + fixtype;
	sPort_send_data(uart, SMARTPORT_ID_DIY_GPS_FIX, t2);
}


/*
 * Sends nav_state
 */
void sPort_send_NAV_STATE(int uart)
{
	uint32_t navstate = (int)(vehicle_status->nav_state);
	sPort_send_data(uart, SMARTPORT_ID_DIY_NAV_STATE, navstate);
}

/*
 * Sends arming_state
 */

void sPort_send_ARMING_STATE(int uart)
{
	uint32_t armingstate = (int)(vehicle_status->arming_state);
	sPort_send_data(uart, SMARTPORT_ID_DIY_ARMING_STATE, armingstate);
}

void sPort_send_ATTITUDE_ROLL(int uart)
{
	uint32_t roll = roundf(vehicle_attitude->roll * 1000.0f);
	sPort_send_data(uart, SMARTPORT_ID_DIY_ATTITUDE_ROLL, roll);
}

void sPort_send_ATTITUDE_PITCH(int uart)
{
	uint32_t pitch = roundf(vehicle_attitude->pitch * 1000.0f);
	sPort_send_data(uart, SMARTPORT_ID_DIY_ATTITUDE_PITCH, pitch);
}

void sPort_send_ATTITUDE_YAW(int uart)
{
	uint32_t yaw = roundf(vehicle_attitude->yaw * 1000.0f);
	sPort_send_data(uart, SMARTPORT_ID_DIY_ATTITUDE_YAW, yaw);
}

void sPort_send_MISSION_SEQUENCE_CURRENT(int uart)
{
	uint32_t seq_current = mission_result->seq_current;
	sPort_send_data(uart, SMARTPORT_ID_DIY_MISSION_SEQUENCE_CURRENT, seq_current);
}

void sPort_send_MISSION_SEQUENCE_REACHED(int uart)
{
	uint32_t seq_reached = mission_result->seq_reached;
	sPort_send_data(uart, SMARTPORT_ID_DIY_MISSION_SEQUENCE_REACHED, seq_reached);
}

void sPort_send_MISSION_SEQUENCE_STATUS(int uart)
{
	uint32_t seq_status =
		(mission_result->valid << 7)
			 | mission_result->warning << 6
			 | mission_result->reached << 5
			 | mission_result->finished << 4
			 | mission_result->stay_in_failsafe << 3
			 | mission_result->flight_termination << 2
			 | mission_result->item_do_jump_changed << 1
			 | mission_result->mission_failure;
	sPort_send_data(uart, SMARTPORT_ID_DIY_MISSION_SEQUENCE_STATUS, seq_status);
}

void sPort_send_MAVLINK_MESSAGE(int uart)
{
	/* from mavlink_log.msg: uint8[50] text.
	 * each character is 8 bit or 1 byte.
	 * which means messages are worst case 400 Bit -> 12.5 * uint32_t
	 * so the text buffer is at maximum 416 Bit = 13 * uint32_t
	 * the first 8 bit and last 8 bit will be used for the start (0x02) and end (0x03) frame.
	 * at a rate of transimitting 20 Hz, we could theoretically transmit 20 uint_32_t per second
	 * so one message should theoretically transmit in under a second.
	 * maximum severity we will send is MAV_SEVERITY_NOTICE = 5
	 * LUA INFO:
	 *	string.format("%c", 0xFF)  will convert from hex to ascii char
	 *  if another startbyte is found without an endbyte, rest of message got lost.
	 *
	 * logic:
	 * if new message and last message sucessfully transmitted (bytes_to_send queue is empty) then
	 * 	->> convert message to sequence of bytes with start and stop frame, push to a bytes_to_send queue
	 * pop one byte from bytes_to_send queue and send
	 */

	//check if there's a mavlink_log message to send in the queue
	uint8_t byte_element = 0;

	if (!fifo_pop(&byte_element)) {
		//Send one uint_32_t consiting of 4 x uint8_t
		//So we need to take 4 bytes and combine them
		uint8_t first_byte = byte_element;
		uint8_t second_byte;
		uint8_t third_byte;
		uint8_t fourth_byte;

		second_byte = (!fifo_pop(&second_byte)) ? second_byte : 0;
		third_byte = (!fifo_pop(&third_byte)) ? third_byte : 0;
		fourth_byte = (!fifo_pop(&fourth_byte)) ? fourth_byte : 0;

		uint32_t byte_to_send = (first_byte << 24) | second_byte << 16 | third_byte << 8 | fourth_byte;
		sPort_send_data(uart, SMARTPORT_ID_DIY_MAVLINK_MESSAGE_BYTE, byte_to_send);
	}
}

void fifo_init(void)
{
	fifoIn = fifoOut = 0;
}

int fifo_push(uint8_t new)
{
	if (fifoIn == ((fifoOut - 1 + FIFO_SIZE) % FIFO_SIZE)) {
		return -1; /* fifo full*/
	}

	fifo[fifoIn] = new;
	fifoIn = (fifoIn + 1) % FIFO_SIZE;
	return 0;
}

int fifo_pop(uint8_t *old)
{
	if (fifoIn == fifoOut) {
		return -1; /* Queue Empty - nothing to get*/
	}

	*old = fifo[fifoOut];
	fifoOut = (fifoOut + 1) % FIFO_SIZE;

	return 0;
}
