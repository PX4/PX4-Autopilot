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
#include <v1.0/common/mavlink.h>
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

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#include "waypoints.h"
#include "mavlink_log.h"
#include "orb_topics.h"
#include "missionlib.h"
#include "mavlink_hil.h"
#include "util.h"
#include "waypoints.h"
#include "mavlink_parameters.h"

static uint8_t missionlib_msg_buf[MAVLINK_MAX_PACKET_LEN];

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
	static orb_advert_t local_position_setpoint_pub = -1;
	char buf[50] = {0};

	/* Update controller setpoints */
	if (frame == (int)MAV_FRAME_GLOBAL) {
		/* global, absolute waypoint */
		struct vehicle_global_position_setpoint_s sp;
		sp.lat = param5_lat_x * 1e7f;
		sp.lon = param6_lon_y * 1e7f;
		sp.altitude = param7_alt_z;
		sp.altitude_is_relative = false;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
		/* Initialize publication if necessary */
		if (global_position_setpoint_pub < 0) {
			global_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &sp);
		} else {
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &sp);
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
	}
	
	mavlink_missionlib_send_gcs_string(buf);
	printf("%s\n", buf);
	//printf("[mavlink mp] new setpoint\n");//: frame: %d, lat: %d, lon: %d, alt: %d, yaw: %d\n", frame, param5_lat_x*1000, param6_lon_y*1000, param7_alt_z*1000, param4*1000);
}
