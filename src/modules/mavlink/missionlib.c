/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file missionlib.h
 * MAVLink missionlib components
 */

// XXX trim includes
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <mavlink/mavlink_log.h>

#include "waypoints.h"
#include "orb_topics.h"
#include "missionlib.h"
#include "mavlink_hil.h"
#include "util.h"
#include "waypoints.h"
#include "mavlink_parameters.h"

static uint8_t missionlib_msg_buf[MAVLINK_MAX_PACKET_LEN];
static uint64_t loiter_start_time;

static bool set_special_fields(float param1, float param2, float param3, float param4, uint16_t command,
	struct vehicle_global_position_setpoint_s *sp);

int
mavlink_missionlib_send_message(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(missionlib_msg_buf, msg);

	mavlink_send_uart_bytes(chan, missionlib_msg_buf, len);
	return 0;
}

int
mavlink_missionlib_send_gcs_string(const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;
	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0')
			break;

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';
		mavlink_message_t msg;

		mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &statustext);
		return mavlink_missionlib_send_message(&msg);

	} else {
		return 1;
	}
}

/**
 * Get system time since boot in microseconds
 *
 * @return the system time since boot in microseconds
 */
uint64_t mavlink_missionlib_get_system_timestamp()
{
	return hrt_absolute_time();
}

/**
 * Set special vehicle setpoint fields based on current mission item.
 *
 * @return true if the mission item could be interpreted
 * successfully, it return false on failure.
 */
bool set_special_fields(float param1, float param2, float param3, float param4, uint16_t command,
	struct vehicle_global_position_setpoint_s *sp)
{
	switch (command) {
		case MAV_CMD_NAV_LOITER_UNLIM:
			sp->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			break;
		case MAV_CMD_NAV_LOITER_TIME:
			sp->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			loiter_start_time = hrt_absolute_time();
			break;
		// case MAV_CMD_NAV_LOITER_TURNS:
		// 	sp->nav_cmd = NAV_CMD_LOITER_TURN_COUNT;
		// 	break;
		case MAV_CMD_NAV_WAYPOINT:
			sp->nav_cmd = NAV_CMD_WAYPOINT;
			break;
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			sp->nav_cmd = NAV_CMD_RETURN_TO_LAUNCH;
			break;
		case MAV_CMD_NAV_LAND:
			sp->nav_cmd = NAV_CMD_LAND;
			break;
		case MAV_CMD_NAV_TAKEOFF:
			sp->nav_cmd = NAV_CMD_TAKEOFF;
			break;
		default:
			/* abort */
			return false;
	}

	sp->loiter_radius = param3;
	sp->loiter_direction = (param3 >= 0) ? 1 : -1;

	sp->param1 = param1;
	sp->param2 = param2;
	sp->param3 = param3;
	sp->param4 = param4;


	/* define the turn distance */
	float orbit = 15.0f;

	if (command == (int)MAV_CMD_NAV_WAYPOINT) {

		orbit = param2;

	} else if (command == (int)MAV_CMD_NAV_LOITER_TURNS ||
			command == (int)MAV_CMD_NAV_LOITER_TIME ||
			command == (int)MAV_CMD_NAV_LOITER_UNLIM) {

		orbit = param3;
	} else {

		// XXX set default orbit via param
		// 15 initialized above
	}

	sp->turn_distance_xy = orbit;
	sp->turn_distance_z = orbit;
}

/**
 * This callback is executed each time a waypoint changes.
 *
 * It publishes the vehicle_global_position_setpoint_s or the
 * vehicle_local_position_setpoint_s topic, depending on the type of waypoint
 */
void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command)
{
	static orb_advert_t global_position_setpoint_pub = -1;
	static orb_advert_t global_position_set_triplet_pub = -1;
	static orb_advert_t local_position_setpoint_pub = -1;
	static unsigned last_waypoint_index = -1;
	char buf[50] = {0};

	// XXX include check if WP is supported, jump to next if not

	/* Update controller setpoints */
	if (frame == (int)MAV_FRAME_GLOBAL) {
		/* global, absolute waypoint */
		struct vehicle_global_position_setpoint_s sp;
		sp.lat = param5_lat_x * 1e7f;
		sp.lon = param6_lon_y * 1e7f;
		sp.altitude = param7_alt_z;
		sp.altitude_is_relative = false;
		sp.yaw = _wrap_pi(param4 / 180.0f * M_PI_F);
		set_special_fields(param1, param2, param3, param4, command, &sp);

		/* Initialize setpoint publication if necessary */
		if (global_position_setpoint_pub < 0) {
			global_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &sp);

		} else {
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &sp);
		}


		/* fill triplet: previous, current, next waypoint */
		struct vehicle_global_position_set_triplet_s triplet;

		/* current waypoint is same as sp */
		memcpy(&(triplet.current), &sp, sizeof(sp));

		/*
		 * Check if previous WP (in mission, not in execution order) 
		 * is available and identify correct index
		 */
		int last_setpoint_index = -1;
		bool last_setpoint_valid = false;

		if (index > 0) {
			last_setpoint_index = index - 1;
		}

		while (last_setpoint_index >= 0) {

			if (wpm->waypoints[last_setpoint_index].frame == (int)MAV_FRAME_GLOBAL &&
				(wpm->waypoints[last_setpoint_index].command == (int)MAV_CMD_NAV_WAYPOINT ||
				wpm->waypoints[last_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_TURNS ||
				wpm->waypoints[last_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_TIME ||
				wpm->waypoints[last_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_UNLIM)) {
				last_setpoint_valid = true;
				break;
			}

			last_setpoint_index--;
		}

		/*
		 * Check if next WP (in mission, not in execution order) 
		 * is available and identify correct index
		 */
		int next_setpoint_index = -1;
		bool next_setpoint_valid = false;

		/* next waypoint */
		if (wpm->size > 1) {
			next_setpoint_index = index + 1;
		}

		while (next_setpoint_index < wpm->size - 1) {

			if (wpm->waypoints[next_setpoint_index].frame == (int)MAV_FRAME_GLOBAL && (wpm->waypoints[next_setpoint_index].command == (int)MAV_CMD_NAV_WAYPOINT ||
					wpm->waypoints[next_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_TURNS ||
					wpm->waypoints[next_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_TIME ||
					wpm->waypoints[next_setpoint_index].command == (int)MAV_CMD_NAV_LOITER_UNLIM)) {
				next_setpoint_valid = true;
				break;
			}

			next_setpoint_index++;
		}

		/* populate last and next */

		triplet.previous_valid = false;
		triplet.next_valid = false;

		if (last_setpoint_valid) {
			triplet.previous_valid = true;
			struct vehicle_global_position_setpoint_s sp;
			sp.lat = wpm->waypoints[last_setpoint_index].x * 1e7f;
			sp.lon = wpm->waypoints[last_setpoint_index].y * 1e7f;
			sp.altitude = wpm->waypoints[last_setpoint_index].z;
			sp.altitude_is_relative = false;
			sp.yaw = (wpm->waypoints[last_setpoint_index].param4 / 180.0f) * M_PI_F - M_PI_F;
			set_special_fields(wpm->waypoints[last_setpoint_index].param1,
				wpm->waypoints[last_setpoint_index].param2,
				wpm->waypoints[last_setpoint_index].param3,
				wpm->waypoints[last_setpoint_index].param4,
				wpm->waypoints[last_setpoint_index].command, &sp);
			memcpy(&(triplet.previous), &sp, sizeof(sp));
		}

		if (next_setpoint_valid) {
			triplet.next_valid = true;
			struct vehicle_global_position_setpoint_s sp;
			sp.lat = wpm->waypoints[next_setpoint_index].x * 1e7f;
			sp.lon = wpm->waypoints[next_setpoint_index].y * 1e7f;
			sp.altitude = wpm->waypoints[next_setpoint_index].z;
			sp.altitude_is_relative = false;
			sp.yaw = (wpm->waypoints[next_setpoint_index].param4 / 180.0f) * M_PI_F - M_PI_F;
			set_special_fields(wpm->waypoints[next_setpoint_index].param1,
				wpm->waypoints[next_setpoint_index].param2,
				wpm->waypoints[next_setpoint_index].param3,
				wpm->waypoints[next_setpoint_index].param4,
				wpm->waypoints[next_setpoint_index].command, &sp);
			memcpy(&(triplet.next), &sp, sizeof(sp));
		}

		/* Initialize triplet publication if necessary */
		if (global_position_set_triplet_pub < 0) {
			global_position_set_triplet_pub = orb_advertise(ORB_ID(vehicle_global_position_set_triplet), &triplet);

		} else {
			orb_publish(ORB_ID(vehicle_global_position_set_triplet), global_position_set_triplet_pub, &triplet);
		}

		sprintf(buf, "[mp] WP#%i lat: % 3.6f/lon % 3.6f/alt % 4.6f/hdg %3.4f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);

	} else if (frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
		/* global, relative alt (in relation to HOME) waypoint */
		struct vehicle_global_position_setpoint_s sp;
		sp.lat = param5_lat_x * 1e7f;
		sp.lon = param6_lon_y * 1e7f;
		sp.altitude = param7_alt_z;
		sp.altitude_is_relative = true;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
		set_special_fields(param1, param2, param3, param4, command, &sp);

		/* Initialize publication if necessary */
		if (global_position_setpoint_pub < 0) {
			global_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &sp);

		} else {
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &sp);
		}



		sprintf(buf, "[mp] WP#%i (lat: %f/lon %f/rel alt %f/hdg %f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);

	} else if (frame == (int)MAV_FRAME_LOCAL_ENU || frame == (int)MAV_FRAME_LOCAL_NED) {
		/* local, absolute waypoint */
		struct vehicle_local_position_setpoint_s sp;
		sp.x = param5_lat_x;
		sp.y = param6_lon_y;
		sp.z = param7_alt_z;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;

		/* Initialize publication if necessary */
		if (local_position_setpoint_pub < 0) {
			local_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &sp);

		} else {
			orb_publish(ORB_ID(vehicle_local_position_setpoint), local_position_setpoint_pub, &sp);
		}

		sprintf(buf, "[mp] WP#%i (x: %f/y %f/z %f/hdg %f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);
	} else {
		warnx("non-navigation WP, ignoring");
		mavlink_missionlib_send_gcs_string("[mp] Unknown waypoint type, ignoring.");
		return;
	}

	/* only set this for known waypoint types (non-navigation types would have returned earlier) */
	last_waypoint_index = index;

	mavlink_missionlib_send_gcs_string(buf);
	printf("%s\n", buf);
	//printf("[mavlink mp] new setpoint\n");//: frame: %d, lat: %d, lon: %d, alt: %d, yaw: %d\n", frame, param5_lat_x*1000, param6_lon_y*1000, param7_alt_z*1000, param4*1000);
}
