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
#include "common.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <lib/geo/geo.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <drivers/drv_hrt.h>

#define frac(f) (f - (int)f)

static constexpr int GPS_MAX_RECEIVERS = 2;

struct s_port_subscription_data_s {
	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<vehicle_acceleration_s> vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionData<vehicle_air_data_s> vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::SubscriptionData<vehicle_global_position_s> vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_gps_position_s> vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_local_position_s> vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<manual_control_setpoint_s> manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionMultiArray<sensor_gps_s> sensor_gps_subs{ORB_ID::sensor_gps};
	sensor_gps_s sensor_gps_data[GPS_MAX_RECEIVERS];
};

static struct s_port_subscription_data_s *s_port_subscription_data = nullptr;


/**
 * Initializes the uORB subscriptions.
 */
bool sPort_init()
{
	s_port_subscription_data = new s_port_subscription_data_s();

	if (s_port_subscription_data == nullptr) {
		return false;
	}

	return true;
}

void sPort_deinit()
{
	if (s_port_subscription_data) {
		delete s_port_subscription_data;
		s_port_subscription_data = nullptr;
	}
}

void sPort_update_topics()
{
	s_port_subscription_data->battery_status_sub.update();
	s_port_subscription_data->vehicle_acceleration_sub.update();
	s_port_subscription_data->vehicle_air_data_sub.update();
	s_port_subscription_data->vehicle_global_position_sub.update();
	s_port_subscription_data->vehicle_gps_position_sub.update();
	s_port_subscription_data->vehicle_local_position_sub.update();
	s_port_subscription_data->vehicle_status_sub.update();
	s_port_subscription_data->manual_control_setpoint_sub.update();

	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		s_port_subscription_data->sensor_gps_subs[i].update(&s_port_subscription_data->sensor_gps_data[i]);
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
	const uint8_t c = 0x10;
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
	uint32_t voltage = (int)(100 * s_port_subscription_data->battery_status_sub.get().voltage_v);
	sPort_send_data(uart, SMARTPORT_ID_VFAS, voltage);
}

// verified scaling
void sPort_send_CUR(int uart)
{
	/* Hijacked to send Data Source type (Mav/RC Control)(David @sees.ai) */
	int16_t control_source = s_port_subscription_data->manual_control_setpoint_sub.get().data_source;
	hrt_abstime control_source_timestamp = s_port_subscription_data->manual_control_setpoint_sub.get().timestamp;
	bool control_source_valid = s_port_subscription_data->manual_control_setpoint_sub.get().valid;

	// If the input has been invalid for >0.5s, then set it to 0
	if ((!control_source_valid) && (hrt_absolute_time() - control_source_timestamp > 500'000)) {
		control_source = manual_control_setpoint_s::SOURCE_UNKNOWN; // This equates to 0
	}

	// If the input type is RC then set it to 1
	// Note - *10 is required
	else if (control_source == manual_control_setpoint_s::SOURCE_RC) {
		control_source = 10 * manual_control_setpoint_s::SOURCE_RC; // This equates to 1
	}

	// Else if the input type is Mavlink then set it to 2
	else if (control_source >= manual_control_setpoint_s::SOURCE_MAVLINK_0
		 && control_source <= manual_control_setpoint_s::SOURCE_MAVLINK_5) {
		control_source = 10 * manual_control_setpoint_s::SOURCE_MAVLINK_0; // This equates to 2
	}

	sPort_send_data(uart, SMARTPORT_ID_CURR, control_source);

	// /* send data */
	// uint32_t current = (int)(10 * s_port_subscription_data->battery_status_sub.get().current_a);
	// sPort_send_data(uart, SMARTPORT_ID_CURR, current);
}

// verified scaling for "custom" altitude option
// OpenTX uses the initial reading as field elevation and displays
// the difference (altitude - field)
void sPort_send_ALT(int uart)
{
	/* send data */
	uint32_t alt = (int)(100 * s_port_subscription_data->vehicle_air_data_sub.get().baro_alt_meter);
	sPort_send_data(uart, SMARTPORT_ID_ALT, alt);
}

// verified scaling for "calculated" option
void sPort_send_SPD(int uart)
{
	const vehicle_local_position_s &local_pos = s_port_subscription_data->vehicle_local_position_sub.get();

	/* send data for A2 */
	float speed = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
	uint32_t ispeed = (int)(10 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

// TODO: verify scaling
void sPort_send_VSPD(int uart, float speed)
{
	/* Hijacked to send additional GPS data (David @sees.ai) */
	// const vehicle_gps_position_s &gps = s_port_subscription_data->vehicle_gps_position_sub.get();
	// s_port_subscription_data->sensor_gps_data[1];
	sensor_gps_s gps_raw;
	s_port_subscription_data->sensor_gps_subs[1].copy(&gps_raw);
	int32_t gps2_fix_type = (int) 100 * gps_raw.fix_type;
	sPort_send_data(uart, SMARTPORT_ID_VARIO, gps2_fix_type);

	/* send data for VARIO vertical speed: int16 cm/sec */
	// int32_t ispeed = (int)(100 * speed);
	// sPort_send_data(uart, SMARTPORT_ID_VARIO, ispeed);
}

// verified scaling
void sPort_send_FUEL(int uart)
{
	/* send data */
	uint32_t fuel = (int)(100 * s_port_subscription_data->battery_status_sub.get().remaining);
	sPort_send_data(uart, SMARTPORT_ID_FUEL, fuel);
}

void sPort_send_GPS_LON(int uart)
{
	/* send longitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	/* precision is approximately 0.1m */
	uint32_t iLon =  6E-2 * fabs(s_port_subscription_data->vehicle_gps_position_sub.get().lon);

	iLon |= (1 << 31);

	if (s_port_subscription_data->vehicle_gps_position_sub.get().lon < 0) { iLon |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLon);
}

void sPort_send_GPS_LAT(int uart)
{
	/* send latitude */
	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 0 and bit 30=sign */
	uint32_t iLat = 6E-2 * fabs(s_port_subscription_data->vehicle_gps_position_sub.get().lat);

	if (s_port_subscription_data->vehicle_gps_position_sub.get().lat < 0) { iLat |= (1 << 30); }

	sPort_send_data(uart, SMARTPORT_ID_GPS_LON_LAT, iLat);
}

void sPort_send_GPS_ALT(int uart)
{
	/* send altitude */
	uint32_t iAlt = s_port_subscription_data->vehicle_gps_position_sub.get().alt / 10;
	sPort_send_data(uart, SMARTPORT_ID_GPS_ALT, iAlt);
}

void sPort_send_GPS_CRS(int uart)
{
	/* send course */

	/* convert to 30 bit signed magnitude degrees*6E5 with MSb = 1 and bit 30=sign */
	int32_t iYaw = s_port_subscription_data->vehicle_local_position_sub.get().heading * 18000.0f / M_PI_F;

	if (iYaw < 0) { iYaw += 36000; }

	sPort_send_data(uart, SMARTPORT_ID_GPS_CRS, iYaw);
}

void sPort_send_GPS_TIME(int uart)
{
	static int date = 0;

	/* send formatted frame */
	time_t time_gps = s_port_subscription_data->vehicle_gps_position_sub.get().time_utc_usec / 1000000ULL;
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
	const vehicle_local_position_s &local_pos = s_port_subscription_data->vehicle_local_position_sub.get();

	/* send 100 * knots */
	float speed = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
	uint32_t ispeed = (int)(1944 * speed);
	sPort_send_data(uart, SMARTPORT_ID_GPS_SPD, ispeed);
}

/*
 * Sends nav_state + 128
 */
void sPort_send_NAV_STATE(int uart)
{
	uint32_t navstate = (int)(128 + s_port_subscription_data->vehicle_status_sub.get().nav_state);

	/* send data */
	sPort_send_data(uart, SMARTPORT_ID_DIY_NAVSTATE, navstate);
}

// verified scaling
// sends number of sats and type of gps fix
void sPort_send_GPS_FIX(int uart)
{
	/* send data */
	uint32_t satcount = (int)(s_port_subscription_data->vehicle_gps_position_sub.get().satellites_used);
	uint32_t fixtype = (int)(s_port_subscription_data->vehicle_gps_position_sub.get().fix_type);
	uint32_t t2 = satcount * 10 + fixtype;
	sPort_send_data(uart, SMARTPORT_ID_DIY_GPSFIX, t2);
}

void sPort_send_flight_mode(int uart)
{
	int16_t telem_flight_mode = get_telemetry_flight_mode(s_port_subscription_data->vehicle_status_sub.get().nav_state);

	sPort_send_data(uart, FRSKY_ID_TEMP1, telem_flight_mode); // send flight mode as TEMP1. This matches with OpenTX & APM
}

void sPort_send_GPS_info(int uart)
{
	sensor_gps_s gps_raw;
	s_port_subscription_data->sensor_gps_subs[0].copy(&gps_raw);
	sPort_send_data(uart, FRSKY_ID_TEMP2, gps_raw.satellites_used * 10 + gps_raw.fix_type);
}



// ---Sees.ai---
// We've added 4 DIY streams to decouple from the vanilla streams and handle the following additional telemetry:
// ID 5002 = GPS1
// ID 5003 = GPS2
// ID 5004 = RC/MAV
// ID 5005 = Flight Mode
// Note: the GPS2 and Flight Mode streams already exist as DIY streams in vanilla PX4 , however for clarity they have
// been duplicated with small sees.ai modifications to structure and naming. These vanilla streams have been disabled at point of transmission.
// In the short-term, the 'hijack' modifications to the vanilla streams have been retained to
// provide cross-compatability with both FRSky controller configurations (Taranis and Horus).
// Once we finalise a single configuration, the 'hijack' mods will be reverted.
void sPort_send_DIY_gps_rov(int uart)
{
	// GPS 1
	sensor_gps_s gps_raw_rov;
	s_port_subscription_data->sensor_gps_subs[0].copy(&gps_raw_rov);
	sPort_send_data(uart, SMARTPORT_ID_ROV_GPS, gps_raw_rov.satellites_used * 10 + gps_raw_rov.fix_type);
}

void sPort_send_DIY_gps_mb(int uart)
{
	// GPS 2
	sensor_gps_s gps_raw_mb;
	s_port_subscription_data->sensor_gps_subs[1].copy(&gps_raw_mb);
	int32_t gps2_fix_type = (int)gps_raw_mb.fix_type;
	sPort_send_data(uart, SMARTPORT_ID_MB_GPS, gps2_fix_type);

}

void sPort_send_DIY_rcmav(int uart)
{
	// OBManual Control Mode
	int16_t control_source = s_port_subscription_data->manual_control_setpoint_sub.get().data_source;
	hrt_abstime control_source_timestamp = s_port_subscription_data->manual_control_setpoint_sub.get().timestamp;
	bool control_source_valid = s_port_subscription_data->manual_control_setpoint_sub.get().valid;

	// If the input has been invalid for >0.5s, then set it to 0
	if ((!control_source_valid) && (hrt_absolute_time() - control_source_timestamp > 500'000)) {
		control_source =
			manual_control_setpoint_s::SOURCE_UNKNOWN; // This equates to 0 and in this case indicates no valid setpoint.
	}

	// If the input type is RC then set it to 1
	else if (control_source == manual_control_setpoint_s::SOURCE_RC) {
		control_source = manual_control_setpoint_s::SOURCE_RC; // This equates to 1
	}

	// Else if the input type is Mavlink then set it to 2
	else if (control_source >= manual_control_setpoint_s::SOURCE_MAVLINK_0
		 && control_source <= manual_control_setpoint_s::SOURCE_MAVLINK_5) {
		control_source = manual_control_setpoint_s::SOURCE_MAVLINK_0; // This equates to 2
	}

	sPort_send_data(uart, SMARTPORT_ID_RCMAV, control_source);
}

void sPort_send_DIY_flgt_mode(int uart)
{
	// Flight Mode
	int16_t telem_flight_mode = get_telemetry_flight_mode(s_port_subscription_data->vehicle_status_sub.get().nav_state);
	sPort_send_data(uart, SMARTPORT_ID_FLGT_MODE,
			telem_flight_mode); // send flight mode as TEMP1. This matches with OpenTX & APM
}
