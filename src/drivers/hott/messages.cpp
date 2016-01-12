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
#include <stdio.h>
#include <string.h>
#include <geo/geo.h>
#include <unistd.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <drivers/drv_hrt.h>

/* The board is very roughly 5 deg warmer than the surrounding air */
#define BOARD_TEMP_OFFSET_DEG 5

static int _battery_sub = -1;
static int _gps_sub = -1;
static int _home_sub = -1;
static int _sensor_sub = -1;
static int _airspeed_sub = -1;
static int _esc_sub = -1;

static orb_advert_t _esc_pub = nullptr;

static bool _home_position_set = false;
static double _home_lat = 0.0d;
static double _home_lon = 0.0d;


void
init_sub_messages(void)
{
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_home_sub = orb_subscribe(ORB_ID(home_position));
	_sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_esc_sub = orb_subscribe(ORB_ID(esc_status));
}

void init_pub_messages(void)
{
}

void
build_gam_request(uint8_t *buffer, size_t *size)
{
	struct gam_module_poll_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.mode = BINARY_REQUEST_ID;
	msg.id = GAM_SENSOR_ID;

	memcpy(buffer, &msg, *size);
}

void
publish_gam_message(const uint8_t *buffer)
{
	struct gam_module_msg msg;
	size_t size = sizeof(msg);
	memset(&msg, 0, size);
	memcpy(&msg, buffer, size);
	struct esc_status_s esc;
	memset(&esc, 0, sizeof(esc));

	// Publish it.
	esc.timestamp = hrt_absolute_time();
	esc.esc_count = 1;
	esc.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_PPM;

	esc.esc[0].esc_vendor = esc_status_s::ESC_VENDOR_GRAUPNER_HOTT;
	esc.esc[0].esc_rpm = (uint16_t)((msg.rpm_H << 8) | (msg.rpm_L & 0xff)) * 10;
	esc.esc[0].esc_temperature = static_cast<float>(msg.temperature1) - 20.0F;
	esc.esc[0].esc_voltage = static_cast<float>((msg.main_voltage_H << 8) | (msg.main_voltage_L & 0xff)) * 0.1F;
	esc.esc[0].esc_current = static_cast<float>((msg.current_H << 8) | (msg.current_L & 0xff)) * 0.1F;

	/* announce the esc if needed, just publish else */
	if (_esc_pub != nullptr) {
		orb_publish(ORB_ID(esc_status), _esc_pub, &esc);

	} else {
		_esc_pub = orb_advertise(ORB_ID(esc_status), &esc);
	}
}

void
build_eam_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the current sensor values */
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	orb_copy(ORB_ID(sensor_combined), _sensor_sub, &raw);

	/* get a local copy of the battery data */
	struct battery_status_s battery;
	memset(&battery, 0, sizeof(battery));
	orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

	struct eam_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;
	msg.eam_sensor_id = EAM_SENSOR_ID;
	msg.sensor_text_id = EAM_SENSOR_TEXT_ID;

	msg.temperature1 = (uint8_t)(20 + 20);	// 20c
	msg.temperature2 = msg.temperature1 - BOARD_TEMP_OFFSET_DEG;

	msg.main_voltage_L = (uint8_t)(5.0 * 10);	//5v

	uint16_t alt = (uint16_t)(22 + 500);	// 22m
	msg.altitude_L = (uint8_t)alt & 0xff;
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

	uint16_t speed = (uint16_t)(1 * 3.6f);	// 1 m/s
	msg.speed_L = (uint8_t)speed & 0xff;
	msg.speed_H = (uint8_t)(speed >> 8) & 0xff;

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
build_gam_response(uint8_t *buffer, size_t *size)
{
	/* get a local copy of the ESC Status values */
	struct esc_status_s esc;
	memset(&esc, 0, sizeof(esc));
	orb_copy(ORB_ID(esc_status), _esc_sub, &esc);

	struct gam_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;
	msg.gam_sensor_id = GAM_SENSOR_ID;
	msg.sensor_text_id = GAM_SENSOR_TEXT_ID;

	msg.temperature1 = (uint8_t)(60 + 20.0F);	//60c
	msg.temperature2 = 20;  // 0 deg. C.

	const uint16_t voltage = (uint16_t)(20 * 10.0F);	//20v
	msg.main_voltage_L = (uint8_t)voltage & 0xff;
	msg.main_voltage_H = (uint8_t)(voltage >> 8) & 0xff;

	const uint16_t current = (uint16_t)(30 * 10.0F);	//30a
	msg.current_L = (uint8_t)current & 0xff;
	msg.current_H = (uint8_t)(current >> 8) & 0xff;

	const uint16_t rpm = (uint16_t)(4000 * 0.1f);	//4000 rpm
	msg.rpm_L = (uint8_t)rpm & 0xff;
	msg.rpm_H = (uint8_t)(rpm >> 8) & 0xff;

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
convert_to_degrees_minutes_seconds(double val, int *deg, int *min, int *sec)
{
	*deg = (int)val;
	/* get a local copy of the current sensor values */
	//struct sensor_combined_s raw;
	//memset(&raw, 0, sizeof(raw));
	//orb_copy(ORB_ID(sensor_combined), _sensor_sub, &raw);

	/* get a local copy of the battery data */
	//struct vehicle_gps_position_s gps;
	//memset(&gps, 0, sizeof(gps));
	//orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps);

	double delta = val - *deg;
	const double min_d = delta * (double)(60.0);
	*min = (int)min_d;
	delta = min_d - *min;
	*sec = (int)(delta * (double)(10000.0));
}

void
build_gps_response(uint8_t *buffer, size_t *size)
{
	struct gps_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;									/**< START_BYTE */
	msg.sensor_id = GPS_SENSOR_ID;							/**< GPS_SENSOR_ID */
	msg.warning = 0;										/**< 1=A 2=B ... */
	msg.sensor_text_id = GPS_SENSOR_TEXT_ID;				/**< GPS_SENSOR_TEXT_ID */
	msg.alarm_inverse1 = 0;									/**< 01 inverse status */
	msg.alarm_inverse2 = 0;									/**< 00 inverse status status 1 = no GPS Signal */


	msg.gps_num_sat = 15; //gps.satellites_used;

	/* The GPS fix type: 0 = none, 2 = 2D, 3 = 3D */
	msg.gps_fix_char = (uint8_t)(3 + 48);
	msg.gps_fix = (uint8_t)(3 + 48);

	/* No point collecting more data if we don't have a 3D fix yet */
	if (3 > 2) { //gps.fix_type
		/* Current flight direction */
		msg.flight_direction = (uint8_t)(3.14159f * 57.2957795130823f);	// South

		/* GPS speed */
		uint16_t speed = (uint16_t)(2 * 3.6f);	// 2 m/s
		msg.gps_speed_L = (uint8_t)speed & 0xff;
		msg.gps_speed_H = (uint8_t)(speed >> 8) & 0xff;

		/* Get latitude in degrees, minutes and seconds */
		double lat = (double)(1.23456d * 1e-7d);

		/* Set the N or S specifier */
		msg.latitude_ns = 0;

		if (lat < 0) {
			msg.latitude_ns = 1;
			lat = abs(lat);
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
		double lon = (double)(6.54321d * 1e-7d);

		/* Set the E or W specifier */
		msg.longitude_ew = 0;

		if (lon < 0) {
			msg.longitude_ew = 1;
			lon = abs(lon);
		}

		convert_to_degrees_minutes_seconds(lon, &deg, &min, &sec);

		uint16_t lon_min = (uint16_t)(deg * 100 + min);
		msg.longitude_min_L = (uint8_t)lon_min & 0xff;
		msg.longitude_min_H = (uint8_t)(lon_min >> 8) & 0xff;
		uint16_t lon_sec = (uint16_t)(sec);
		msg.longitude_sec_L = (uint8_t)lon_sec & 0xff;
		msg.longitude_sec_H = (uint8_t)(lon_sec >> 8) & 0xff;

		/* Altitude */
		uint16_t alt = (uint16_t)(21 * 1e-3f + 500.0f);	//21m
		msg.altitude_L = (uint8_t)alt & 0xff;
		msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;

		/* Get any (and probably only ever one) _home_sub position report */
		bool updated;
		orb_check(_home_sub, &updated);

		_home_lat = 1.23456;
		_home_lon = 6.43210;
		_home_position_set = true;

		if (updated) {
			/* get a local copy of the home position data */
			struct home_position_s home;
			memset(&home, 0, sizeof(home));
			orb_copy(ORB_ID(home_position), _home_sub, &home);

			_home_lat = home.lat;
			_home_lon = home.lon;
			_home_position_set = true;
		}

		/* Distance from home */
		if (_home_position_set) {
			uint16_t dist = 20; // 20m? (uint16_t)get_distance_to_next_waypoint(_home_lat, _home_lon, lat, lon);

			msg.distance_L = (uint8_t)dist & 0xff;
			msg.distance_H = (uint8_t)(dist >> 8) & 0xff;

			/* Direction back to home */
			uint16_t bearing = 0;
			// North (uint16_t)(get_bearing_to_next_waypoint(_home_lat, _home_lon, lat, lon) * M_RAD_TO_DEG_F);

			msg.home_direction = (uint8_t)bearing >> 1;
		}
	}

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
build_alt_response(uint8_t *buffer, size_t *size)
{
	struct alt_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;									/**< START_BYTE */
	msg.alt_sensor_id = ALT_SENSOR_ID;						/**< ALT_SENSOR_ID */
	//msg.warning = 0;										/**< TODO: figure out codes */
	msg.sensor_text_id = ALT_SENSOR_TEXT_ID;				/**< ALT_SENSOR_TEXT_ID */
	//msg.alarm_invers1 = 0;								/**< Inverse display (alarm?) bitmask */

	uint16_t alt = (uint16_t)(500);
	msg.altitude_L = (uint8_t) alt & 0xff;					/**< Altitude low int8_t. In meters.
																 A value of 500 means 0m */
	msg.altitude_H = (uint8_t)(alt >> 8) & 0xff;			/**< Altitude high int8_t */

	uint16_t alt_max = (uint16_t)(500);
	msg.altitude_max_L = (uint8_t) alt_max & 0xff;			/**< Max. measured altitude low int8_t. In meters.
																 A value of 500 means 0m */
	msg.altitude_max_H = (uint8_t)(alt_max >> 8) & 0xff;	/**< Max. measured altitude high int8_t */

	uint16_t alt_min = (uint16_t)(500);
	msg.altitude_min_L = (uint8_t) alt_min & 0xff;			/**< Min. measured altitude low int8_t. In meters.
																 A value of 500 means 0m */
	msg.altitude_min_H = (uint8_t)(alt_min >> 8) & 0xff;	/**< Min. measured altitude high int8_t */

	uint16_t climb = (uint16_t)(30100);
	msg.climbrate_L = (uint8_t) climb & 0xff;				/**< Climb rate in m/s. Steps of 0.01m/s.
																 Value of 30000 = 0.00 m/s */
	msg.climbrate_H = (uint8_t)(climb >> 8) & 0xff;			/**< Climb rate in m/s */

	uint16_t climb3 = (uint16_t)(30200);
	msg.climbrate3s_L = (uint8_t) climb3 & 0xff;			/**< Climb rate in m/3s. Steps of 0.01m/3s.
																 Value of 30000 = 0.00 m/3s */
	msg.climbrate3s_H = (uint8_t)(climb3 >> 8) & 0xff;		/**< Climb rate m/3s low int8_t */

	uint16_t climb10 = (uint16_t)(30300);
	msg.climbrate10s_L = (uint8_t) climb10 & 0xff;			/**< Climb rate m/10s. Steps of 0.01m/10s.
																 Value of 30000 = 0.00 m/10s */
	msg.climbrate10s_H = (uint8_t)(climb10 >> 8) & 0xff;	/**< Climb rate m/10s low int8_t */

	const char buf[20] = "Testing 1 2 3\0";
	memcpy(msg.text_msg, buf, 14);							/**< Free ASCII text message */

	msg.free_char1 = 'Z';									/**< Free ASCII character.  appears right to home distance */
	msg.free_char2 = 'Y';									/**< Free ASCII character.  appears right to home direction */
	msg.free_char3 = '%';									/**< Free ASCII character.  appears? TODO: Check where this char appears */
	msg.compass_direction = 17;								/**< Compass heading in 2° steps. 1 = 2° */

	//msg.version = 0;										/**< version number TODO: more info? */

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}

void
build_esc_response(uint8_t *buffer, size_t *size)
{
	struct esc_module_msg msg;
	*size = sizeof(msg);
	memset(&msg, 0, *size);

	msg.start = START_BYTE;									/**< START_BYTE */
	msg.esc_sensor_id = ESC_SENSOR_ID;						/**< ESC_SENSOR_ID */
	//msg.warning = 0;										/**< 1=A 2=B ... */
	msg.sensor_text_id = ESC_SENSOR_TEXT_ID;				/**< ESC_SENSOR_TEXT_ID */
	//msg.alarm_invers1 = 0;								/**< TODO: more info */
	//msg.alarm_invers2 = 0;								/**< TODO: more info */

	uint16_t volt = (uint16_t)(500);
	msg.input_v_L = (uint8_t) volt & 0xff;					/**< Input voltage low byte */
	msg.input_v_H = (uint8_t)(volt >> 8) & 0xff;

	uint16_t volt_min = (uint16_t)(500);
	msg.input_v_min_L = (uint8_t) volt_min & 0xff;			/**< Input min. voltage low byte */
	msg.input_v_min_H = (uint8_t)(volt_min >> 8) & 0xff;

	uint16_t batt_cap = (uint16_t)(500);
	msg.batt_cap_L = (uint8_t) batt_cap & 0xff;				/**< battery capacity in 10mAh steps */
	msg.batt_cap_H = (uint8_t)(batt_cap >> 8) & 0xff;

	//msg.esc_temp = 0;										/**< ESC temperature */
	//msg.esc_max_temp = 0;									/**< ESC max. temperature */

	uint16_t curr = (uint16_t)(50);
	msg.current_L = (uint8_t) curr & 0xff;					/**< Current in 0.1 steps */
	msg.current_H = (uint8_t)(curr >> 8) & 0xff;

	uint16_t curr_max = (uint16_t)(55);
	msg.current_max_L = (uint8_t) curr_max & 0xff;			/**< Current max. in 0.1 steps */
	msg.current_max_H = (uint8_t)(curr_max >> 8) & 0xff;

	uint16_t rpm = (uint16_t)(100);
	msg.rpm_L = (uint8_t) rpm & 0xff;						/**< RPM in 10U/min steps */
	msg.rpm_H = (uint8_t)(rpm >> 8) & 0xff;

	uint16_t rpm_max = (uint16_t)(110);
	msg.rpm_max_L = (uint8_t) rpm_max & 0xff;				/**< RPM max */
	msg.rpm_max_H = (uint8_t)(rpm_max >> 8) & 0xff;

	msg.throttle = 45;										/**< throttle in % */

	uint16_t speed = (uint16_t)(500);
	msg.speed_L = (uint8_t) speed & 0xff;					/**< Speed */
	msg.speed_H = (uint8_t)(speed >> 8) & 0xff;

	uint16_t speed_max = (uint16_t)(500);
	msg.speed_max_L = (uint8_t) speed_max & 0xff;			/**< Speed max */
	msg.speed_max_H = (uint8_t)(speed_max >> 8) & 0xff;

	msg.bec_v = 123;										/**< BEC voltage */
	msg.bec_min_v = 111;									/**< BEC min. voltage */
	msg.bec_current = 234;									/**< BEC current */

	uint16_t becamax = (uint16_t)(100);
	msg.bec_current_max_L = (uint8_t) becamax & 0xff;		/**< BEC max. current */
	msg.bec_current_max_H = (uint8_t)(becamax >> 8) & 0xff;	/**< TODO: not really clear why 2 bytes... */

	//msg.pwm = 0;											/**< PWM */
	//msg.bec_temp = 0;										/**< BEC temperature */
	//msg.bec_temp_max = 0;									/**< BEC highest temperature */
	//msg.motor_temp = 0;									/**< Motor or external sensor temperature */
	//msg.motor_temp_max = 0;								/**< Highest motor or external sensor temperature */

	uint16_t motor_rpm = (uint16_t)(500);
	msg.motor_rpm_L = (uint8_t) motor_rpm & 0xff;			/**< Motor or external RPM sensor (without gear) */
	msg.motor_rpm_H = (uint8_t)(motor_rpm >> 8) & 0xff;

	//msg.motor_timing = 0;									/**< Motor timing */
	//msg.motor_timing_adv = 0;								/**< Motor advanced timing */
	//msg.motor_highest_current = 0;						/**< Motor number (1-x) with highest current */
	//msg.version = 0;										/**< Version number (highest current motor 1-x) */

	msg.stop = STOP_BYTE;
	memcpy(buffer, &msg, *size);
}
