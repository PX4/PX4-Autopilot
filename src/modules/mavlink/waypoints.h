/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
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
 * @file waypoints.h
 * MAVLink waypoint protocol definition (BSD-relicensed).
 */

#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */

#include <v1.0/mavlink_types.h>

// #ifndef MAVLINK_SEND_UART_BYTES
// #define MAVLINK_SEND_UART_BYTES(chan, buffer, len) mavlink_send_uart_bytes(chan, buffer, len)
// #endif
//extern mavlink_system_t mavlink_system;
#include "mavlink_bridge_header.h"
#include <stdbool.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/navigation_capabilities.h>

// FIXME XXX - TO BE MOVED TO XML
enum MAVLINK_WPM_STATES {
	MAVLINK_WPM_STATE_IDLE = 0,
	MAVLINK_WPM_STATE_SENDLIST,
	MAVLINK_WPM_STATE_SENDLIST_SENDWPS,
	MAVLINK_WPM_STATE_GETLIST,
	MAVLINK_WPM_STATE_GETLIST_GETWPS,
	MAVLINK_WPM_STATE_GETLIST_GOTALL,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES {
	MAVLINK_WPM_CODE_OK = 0,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
	MAVLINK_WPM_CODE_ENUM_END
};


/* WAYPOINT MANAGER - MISSION LIB */

#define MAVLINK_WPM_MAX_WP_COUNT 15
#define MAVLINK_WPM_CONFIG_IN_FLIGHT_UPDATE				  ///< Enable double buffer and in-flight updates
#ifndef MAVLINK_WPM_TEXT_FEEDBACK
#define MAVLINK_WPM_TEXT_FEEDBACK 0						  ///< Report back status information as text
#endif
#define MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT 5000000         ///< Protocol communication timeout in useconds
#define MAVLINK_WPM_SETPOINT_DELAY_DEFAULT 1000000           ///< When to send a new setpoint
#define MAVLINK_WPM_PROTOCOL_DELAY_DEFAULT 40000


struct mavlink_wpm_storage {
	mavlink_mission_item_t waypoints[MAVLINK_WPM_MAX_WP_COUNT];      ///< Currently active waypoints
#ifdef MAVLINK_WPM_CONFIG_IN_FLIGHT_UPDATE
	mavlink_mission_item_t rcv_waypoints[MAVLINK_WPM_MAX_WP_COUNT];  ///< Receive buffer for next waypoints
#endif
	uint16_t size;
	uint16_t max_size;
	uint16_t rcv_size;
	enum MAVLINK_WPM_STATES current_state;
	int16_t current_wp_id;							///< Waypoint in current transmission
	int16_t current_active_wp_id;					///< Waypoint the system is currently heading towards
	uint16_t current_count;
	uint8_t current_partner_sysid;
	uint8_t current_partner_compid;
	uint64_t timestamp_lastaction;
	uint64_t timestamp_last_send_setpoint;
	uint64_t timestamp_firstinside_orbit;
	uint64_t timestamp_lastoutside_orbit;
	uint32_t timeout;
	uint32_t delay_setpoint;
	float accept_range_yaw;
	float accept_range_distance;
	bool yaw_reached;
	bool pos_reached;
	bool idle;
};

typedef struct mavlink_wpm_storage mavlink_wpm_storage;

void mavlink_wpm_init(mavlink_wpm_storage *state);
int mavlink_waypoint_eventloop(uint64_t now, const struct vehicle_global_position_s *global_position,
			       struct vehicle_local_position_s *local_pos, struct navigation_capabilities_s *nav_cap);
void mavlink_wpm_message_handler(const mavlink_message_t *msg, const struct vehicle_global_position_s *global_pos ,
				 struct vehicle_local_position_s *local_pos);

extern void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command);

#endif /* WAYPOINTS_H_ */
