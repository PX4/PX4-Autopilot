/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file waypoints.c
 * MAVLink waypoint protocol implementation (BSD-relicensed).
 */

#include <math.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <stdio.h>

#include "mavlink/mavlink_bridge_header.h"
#include "mavlink/missionlib.h"
#include "uORB/uORB.h"
#include "mavlink/orb_topics.h"
#include "waypoints.h"
#include "dataman/dataman.h"
#include "lib/geo/geo.h"

#ifndef FM_PI
#define FM_PI 3.1415926535897932384626433832795d
#endif

bool debug = false;
bool verbose = false;

static uint64_t loiter_start_time;

#define WPM_NO_PRINTF
#define WPM_VERBOSE 0

uint8_t mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;

orb_advert_t mission_pub = -1;

/* Allocate storage space for waypoints */
__EXPORT waypoints_storage wpm;

void get_waypoint(int ix, mission_item_t *wp)
{
	dm_read(DM_KEY_WAY_POINTS, ix, wp, sizeof(mission_item_t));
}

void set_waypoint(int ix, mission_item_t *wp)
{
	dm_write(DM_KEY_WAY_POINTS, ix, DM_PERSIST_IN_FLIGHT_RESET, wp, sizeof(mission_item_t));
}


__EXPORT void waypoints_init(waypoints_storage *state)
{
	// Set all waypoints to zero

	// Set count to zero
	state->size = 0;
	state->max_size = MAX_MISSION_ITEMS;
	state->current_state = WPM_STATE_IDLE;
	state->current_partner_sysid = 0;
	state->current_partner_compid = 0;
	state->timestamp_lastaction = 0;
	state->timestamp_last_send_setpoint = 0;
	state->timeout = WPM_PROTOCOL_TIMEOUT_DEFAULT;
	state->delay_setpoint = WPM_SETPOINT_DELAY_DEFAULT;
	state->idle = false;      				///< indicates if the system is following the waypoints or is waiting
	state->current_active_wp_id = -1;		///< id of current waypoint
	state->yaw_reached = false;						///< boolean for yaw attitude reached
	state->pos_reached = false;						///< boolean for position reached
	state->timestamp_lastoutside_orbit = 0;///< timestamp when the MAV was last outside the orbit or had the wrong yaw value
	state->timestamp_firstinside_orbit = 0;///< timestamp when the MAV was the first time after a waypoint change inside the orbit and had the correct yaw value

}

/*
 *  @brief Sends an waypoint ack message
 */
static void wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_mission_ack_t wpa;

	wpa.target_system = wpm.current_partner_sysid;
	wpa.target_component = wpm.current_partner_compid;
	wpa.type = type;

	mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpa);
	missionlib_send_mavlink_message(&msg);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	if (WPM_TEXT_FEEDBACK) {
#ifndef WPM_NO_PRINTF
		if (WPM_VERBOSE) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);
#endif
		missionlib_send_mavlink_gcs_string("Sent waypoint ACK");
	}
}

/*
 *  @brief Broadcasts the new target waypoint and directs the MAV to fly there
 *
 *  This function broadcasts its new active waypoint sequence number and
 *  sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *
 *  @param seq The waypoint sequence number the MAV should fly to.
 */
static void wpm_send_waypoint_current(uint16_t seq)
{
	if (seq < wpm.size) {
		mission_item_t cur;
		get_waypoint(seq, &cur);

		mavlink_message_t msg;
		mavlink_mission_current_t wpc;

		wpc.seq = cur.seq;

		mavlink_msg_mission_current_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
		missionlib_send_mavlink_message(&msg);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("Set current waypoint\n"); //// printf("Broadcasted new current waypoint %u\n", wpc.seq);

	} else {
		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("ERROR: wp index out of bounds\n");
	}
}

/*
 *  @brief Directs the MAV to fly to a position
 *
 *  Sends a message to the controller, advising it to fly to the coordinates
 *  of the waypoint with a given orientation
 *
 *  @param seq The waypoint sequence number the MAV should fly to.
 */
static void wpm_send_setpoint(uint16_t seq)
{
	if (seq < wpm.size) {
		mission_item_t cur;
		get_waypoint(seq, &cur);
		waypoints_current_waypoint_changed(cur.seq, cur.aceptable_radius,
						   cur.acceptable_time, cur.orbit, cur.yaw, cur.lat,
						   cur.lon, cur.alt, cur.frame, cur.command);

		wpm.timestamp_last_send_setpoint = missionlib_get_system_timestamp();

	} else {
		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("ERROR: Waypoint index out of bounds\n"); //// if (verbose) // printf("ERROR: index out of bounds\n");
	}
}

static void wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_mission_count_t wpc;

	wpc.target_system = wpm.current_partner_sysid;
	wpc.target_component = wpm.current_partner_compid;
	wpc.count = count;

	mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
	missionlib_send_mavlink_message(&msg);

	if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("Sent waypoint count"); //// if (verbose) // printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

static void wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm.size) {
		mavlink_message_t msg;
		mission_item_t wp;
		get_waypoint(seq, &wp);
		mavlink_mission_item_t mwp;
		mwp.param1 = wp.aceptable_radius;
		mwp.param2 = wp.acceptable_time;
		mwp.param3 = wp.orbit;
		mwp.param4 = wp.yaw;
		mwp.x = wp.lat;
		mwp.y = wp.lon;
		mwp.z = wp.alt;
		mwp.seq = wp.seq;
		mwp.command = wp.command;
		mwp.frame = wp.frame;
		mwp.current = wp.current;
		mwp.autocontinue = wp.autocontinue;
		mwp.target_system = wpm.current_partner_sysid;
		mwp.target_component = wpm.current_partner_compid;
		mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &mwp);
		missionlib_send_mavlink_message(&msg);

		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("Sent waypoint"); //// if (verbose) // printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	} else {
		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("ERROR: Waypoint index out of bounds\n");
	}
}

static void wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm.max_size) {
		mavlink_message_t msg;
		mavlink_mission_request_t wpr;
		wpr.target_system = wpm.current_partner_sysid;
		wpr.target_component = wpm.current_partner_compid;
		wpr.seq = seq;
		mavlink_msg_mission_request_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpr);
		missionlib_send_mavlink_message(&msg);

		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("Sent waypoint request"); //// if (verbose) // printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	} else {
		if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("ERROR: Waypoint index exceeds list capacity\n");
	}
}

/*
 *  @brief emits a message that a waypoint reached
 *
 *  This function broadcasts a message that a waypoint is reached.
 *
 *  @param seq The waypoint sequence number the MAV has reached.
 */
static void wpm_send_waypoint_reached(uint16_t seq)
{
	mavlink_message_t msg;
	mavlink_mission_item_reached_t wp_reached;

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wp_reached);
	missionlib_send_mavlink_message(&msg);

	if (WPM_TEXT_FEEDBACK) missionlib_send_mavlink_gcs_string("Sent waypoint reached message"); //// if (verbose) // printf("Sent waypoint %u reached message\n", wp_reached.seq);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

/*
 * Calculate distance in global frame.
 *
 * The distance calculation is based on the WGS84 geoid (GPS)
 */
static float wpm_distance_to_point_global_wgs84(uint16_t seq, float lat, float lon, float alt, float *dist_xy, float *dist_z)
{

	if (seq < wpm.size) {
		mission_item_t wp;
		get_waypoint(seq, &wp);

		double current_x_rad = wp.lat / 180.0 * M_PI;
		double current_y_rad = wp.lon / 180.0 * M_PI;
		double x_rad = lat / 180.0 * M_PI;
		double y_rad = lon / 180.0 * M_PI;

		double d_lat = x_rad - current_x_rad;
		double d_lon = y_rad - current_y_rad;

		double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0f) * cos(current_x_rad) * cos(x_rad);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));

		const double radius_earth = 6371000.0f;

		float dxy = radius_earth * c;
		float dz = alt - wp.alt;

		*dist_xy = fabsf(dxy);
		*dist_z = fabsf(dz);

		return sqrtf(dxy * dxy + dz * dz);

	} else {
		return -1.0f;
	}

}

/*
 * Calculate distance in local frame (NED)
 */
static float wpm_distance_to_point_local(uint16_t seq, float x, float y, float z, float *dist_xy, float *dist_z)
{
	if (seq < wpm.size) {
		mission_item_t cur;
		get_waypoint(seq, &cur);

		float dx = (cur.lat - x);
		float dy = (cur.lon - y);
		float dz = (cur.alt - z);

		*dist_xy = sqrtf(dx * dx + dy * dy);
		*dist_z = fabsf(dz);

		return sqrtf(dx * dx + dy * dy + dz * dz);

	} else {
		return -1.0f;
	}
}

static void check_waypoints_reached(uint64_t now, const struct vehicle_global_position_s *global_pos, float turn_distance)
{
	static uint16_t counter;

	if ((!global_pos->valid) || wpm.size == 0) {
		/* nothing to check here, return */
		return;
	}

	if (wpm.current_active_wp_id < wpm.size) {

		float orbit;
		mission_item_t wp;
		get_waypoint(wpm.current_active_wp_id, &wp);

		if (wp.command == MAV_CMD_NAV_WAYPOINT) {

			orbit = wp.acceptable_time;

		} else if (wp.command == MAV_CMD_NAV_LOITER_TURNS ||
			   wp.command == MAV_CMD_NAV_LOITER_TIME ||
			   wp.command == MAV_CMD_NAV_LOITER_UNLIM) {

			orbit = wp.orbit;

		} else {

			// XXX set default orbit via param
			orbit = 15.0f;
		}

		/* keep vertical orbit */
		float vertical_switch_distance = orbit;

		/* Take the larger turn distance - orbit or turn_distance */
		if (orbit < turn_distance)
			orbit = turn_distance;

		int coordinate_frame = wp.frame;
		float dist = -1.0f;

		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		if (coordinate_frame == (int)MAV_FRAME_GLOBAL) {
			dist = wpm_distance_to_point_global_wgs84(wpm.current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->alt, &dist_xy, &dist_z);

		} else if (coordinate_frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
			dist = wpm_distance_to_point_global_wgs84(wpm.current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->relative_alt, &dist_xy, &dist_z);

		//} else if (coordinate_frame == (int)MAV_FRAME_LOCAL_ENU || coordinate_frame == (int)MAV_FRAME_LOCAL_NED) {
		//	dist = wpm_distance_to_point_local(dm, wpm.current_active_wp_id, local_pos->x, local_pos->y, local_pos->z, &dist_xy, &dist_z);

		} else if (coordinate_frame == (int)MAV_FRAME_MISSION) {
			/* Check if conditions of mission item are satisfied */
			// XXX TODO
		}

		if (dist >= 0.f && dist_xy <= orbit && dist_z >= 0.0f && dist_z <= vertical_switch_distance) {
			wpm.pos_reached = true;
		}

		// check if required yaw reached
		float yaw_sp = _wrap_pi(wp.yaw / 180.0f * FM_PI);
		float yaw_err = _wrap_pi(yaw_sp - global_pos->yaw);

		if (fabsf(yaw_err) < 0.05f) {
			wpm.yaw_reached = true;
		}
	}

	//check if the current waypoint was reached
	if (wpm.pos_reached && /*wpm.yaw_reached &&*/ !wpm.idle) {
		if (wpm.current_active_wp_id < wpm.size) {
			mission_item_t cur_wp;
			get_waypoint(wpm.current_active_wp_id, &cur_wp);

			if (wpm.timestamp_firstinside_orbit == 0) {
				// Announce that last waypoint was reached
				wpm_send_waypoint_reached(cur_wp.seq);
				wpm.timestamp_firstinside_orbit = now;
			}

			// check if the MAV was long enough inside the waypoint orbit
			//if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))

			bool time_elapsed = false;

			if (now - wpm.timestamp_firstinside_orbit >= cur_wp.aceptable_radius * 1000 * 1000) {
				time_elapsed = true;

			} else if (cur_wp.command == MAV_CMD_NAV_TAKEOFF) {
				time_elapsed = true;
			}

			if (time_elapsed) {

				if (cur_wp.autocontinue) {
					cur_wp.current = 0;
					set_waypoint(wpm.current_active_wp_id, &cur_wp);

					float navigation_lat = -1.0f;
					float navigation_lon = -1.0f;
					float navigation_alt = -1.0f;
					int navigation_frame = -1;

					/* initialize to current position in case we don't find a suitable navigation waypoint */
					if (global_pos->valid) {
						navigation_lat = global_pos->lat / 1e7;
						navigation_lon = global_pos->lon / 1e7;
						navigation_alt = global_pos->alt;
						navigation_frame = MAV_FRAME_GLOBAL;
					}

					//else if (local_pos->xy_valid && local_pos->z_valid) {
					//	navigation_lat = local_pos->x;
					//	navigation_lon = local_pos->y;
					//	navigation_alt = local_pos->z;
					//	navigation_frame = MAV_FRAME_LOCAL_NED;
					//}

					/* only accept supported navigation waypoints, skip unknown ones */
					do {
						get_waypoint(wpm.current_active_wp_id, &cur_wp);

						/* pick up the last valid navigation waypoint, this will be one we hold on to after the mission */
						if (cur_wp.command == MAV_CMD_NAV_WAYPOINT ||
						    cur_wp.command == MAV_CMD_NAV_LOITER_TURNS ||
						    cur_wp.command == MAV_CMD_NAV_LOITER_TIME ||
						    cur_wp.command == MAV_CMD_NAV_LOITER_UNLIM) {

							/* this is a navigation waypoint */
							navigation_frame = cur_wp.frame;
							navigation_lat = cur_wp.lat;
							navigation_lon = cur_wp.lon;
							navigation_alt = cur_wp.alt;
						}

						if (wpm.current_active_wp_id == wpm.size - 1 && wpm.size > 1) {
							/* the last waypoint was reached, if auto continue is
							 * activated keep the system loitering there.
							 */
							cur_wp.command = MAV_CMD_NAV_LOITER_UNLIM;
							cur_wp.orbit = 20.0f; // XXX magic number 20 m loiter radius
							cur_wp.frame = navigation_frame;
							cur_wp.lat = navigation_lat;
							cur_wp.lon = navigation_lon;
							cur_wp.alt = navigation_alt;
							cur_wp.autocontinue = false;
							set_waypoint(wpm.current_active_wp_id, &cur_wp);

							/* we risk an endless loop for missions without navigation waypoints, abort. */
							break;

						} else {
							if ((uint16_t)(wpm.current_active_wp_id + 1) < wpm.size)
								wpm.current_active_wp_id++;
						}

					} while (!(cur_wp.command == (int)MAV_CMD_NAV_WAYPOINT ||
						   cur_wp.command == (int)MAV_CMD_NAV_LOITER_TURNS ||
						   cur_wp.command == (int)MAV_CMD_NAV_LOITER_TIME ||
						   cur_wp.command == (int)MAV_CMD_NAV_LOITER_UNLIM));

					// Fly to next waypoint
					wpm.timestamp_firstinside_orbit = 0;
					wpm_send_waypoint_current(wpm.current_active_wp_id);
					wpm_send_setpoint(wpm.current_active_wp_id);
					cur_wp.current = 1;
					set_waypoint(wpm.current_active_wp_id, &cur_wp);
					wpm.pos_reached = false;
					wpm.yaw_reached = false;
					printf("Set new waypoint (%u)\n", wpm.current_active_wp_id);
				}
			}
		}

	} else {
		wpm.timestamp_lastoutside_orbit = now;
	}

	counter++;
}


__EXPORT int waypoints_check(uint64_t now, const struct vehicle_global_position_s *global_position, struct navigation_capabilities_s *nav_cap)
{
	/* check for timed-out operations */
	if (now - wpm.timestamp_lastaction > wpm.timeout && wpm.current_state != WPM_STATE_IDLE) {

#ifndef WPM_NO_PRINTF
		if (WPM_VERBOSE) printf("Last operation (state=%u) timed out, changing state to WPM_STATE_IDLE\n", wpm.current_state);
#endif
                missionlib_send_mavlink_gcs_string("Operation timeout switching -> IDLE");
		wpm.current_state = WPM_STATE_IDLE;
		wpm.current_count = 0;
		wpm.current_partner_sysid = 0;
		wpm.current_partner_compid = 0;
		wpm.current_wp_id = -1;

		if (wpm.size == 0) {
			wpm.current_active_wp_id = -1;
		}
	}

	check_waypoints_reached(now, global_position, nav_cap->turn_distance);

	return OK;
}


__EXPORT void
waypoints_message_handler(const mavlink_message_t *msg, const struct vehicle_global_position_s *global_pos , struct vehicle_local_position_s *local_pos)
{
	uint64_t now = missionlib_get_system_timestamp();

	switch (msg->msgid) {

	case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				wpm.timestamp_lastaction = now;

				if (wpm.current_state == WPM_STATE_SENDLIST || wpm.current_state == WPM_STATE_SENDLIST_SENDWPS) {
					if (wpm.current_wp_id == wpm.size - 1) {

						missionlib_send_mavlink_gcs_string("Got last WP ACK state -> IDLE");

						wpm.current_state = WPM_STATE_IDLE;
						wpm.current_wp_id = 0;
					}
				}

			} else {
				missionlib_send_mavlink_gcs_string("REJ. WP CMD: curr partner id mismatch");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
			mavlink_mission_set_current_t wpc;
			mavlink_msg_mission_set_current_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid /*&& wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm.timestamp_lastaction = now;

				if (wpm.current_state == WPM_STATE_IDLE) {
					if (wpc.seq < wpm.size) {
						// if (verbose) // printf("Received MAVLINK_MSG_ID_MISSION_ITEM_SET_CURRENT\n");
						wpm.current_active_wp_id = wpc.seq;
						uint32_t i;

						for (i = 0; i < wpm.size; i++) {
							mission_item_t wp;
							get_waypoint(i, &wp);
							bool current = wp.current;

							if (i == wpm.current_active_wp_id) {
								wp.current = 1;

							} else {
								wp.current = 0;
							}

							if (current ^ (bool)wp.current)
								set_waypoint(i, &wp);
						}

						missionlib_send_mavlink_gcs_string("NEW WP SET");

						wpm.yaw_reached = false;
						wpm.pos_reached = false;
						wpm_send_waypoint_current(wpm.current_active_wp_id);
						wpm_send_setpoint(wpm.current_active_wp_id);
						wpm.timestamp_firstinside_orbit = 0;

					} else {
						missionlib_send_mavlink_gcs_string("IGN WP CURR CMD: Not in list");
					}

				} else {
					missionlib_send_mavlink_gcs_string("IGN WP CURR CMD: Busy");

				}

			} else {
				missionlib_send_mavlink_gcs_string("REJ. WP CMD: target id mismatch");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
			mavlink_mission_request_list_t wprl;
			mavlink_msg_mission_request_list_decode(msg, &wprl);

			if (wprl.target_system == mavlink_system.sysid /*&& wprl.target_component == mavlink_wpm_comp_id*/) {
				wpm.timestamp_lastaction = now;

				if (wpm.current_state == WPM_STATE_IDLE || wpm.current_state == WPM_STATE_SENDLIST) {
					if (wpm.size > 0) {
						//if (verbose && wpm.current_state == WPM_STATE_IDLE) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST from %u changing state to WPM_STATE_SENDLIST\n", msg->sysid);
//                        if (verbose && wpm.current_state == WPM_STATE_SENDLIST) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST again from %u staying in state WPM_STATE_SENDLIST\n", msg->sysid);
						wpm.current_state = WPM_STATE_SENDLIST;
						wpm.current_wp_id = 0;
						wpm.current_partner_sysid = msg->sysid;
						wpm.current_partner_compid = msg->compid;

					} else {
						// if (verbose) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
					}

					wpm.current_count = wpm.size;
					wpm_send_waypoint_count(msg->sysid, msg->compid, wpm.current_count);

				} else {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST because i'm doing something else already (state=%i).\n", wpm.current_state);
				}
			} else {
				// if (verbose) // printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT MISMATCH\n");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST: {
			mavlink_mission_request_t wpr;
			mavlink_msg_mission_request_decode(msg, &wpr);

			if (msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) {
				wpm.timestamp_lastaction = now;

				//ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				if ((wpm.current_state == WPM_STATE_SENDLIST && wpr.seq == 0) || (wpm.current_state == WPM_STATE_SENDLIST_SENDWPS && (wpr.seq == wpm.current_wp_id || wpr.seq == wpm.current_wp_id + 1) && wpr.seq < wpm.size)) {
					if (wpm.current_state == WPM_STATE_SENDLIST) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u changing state to WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                                                missionlib_send_mavlink_gcs_string("GOT WP REQ, state -> SEND");
					}

					if (wpm.current_state == WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id + 1) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u staying in state WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                                                missionlib_send_mavlink_gcs_string("GOT 2nd WP REQ");
					}

					if (wpm.current_state == WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm.current_wp_id) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u (again) from %u staying in state WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);
#endif
                                                missionlib_send_mavlink_gcs_string("GOT 2nd WP REQ");
					}

					wpm.current_state = WPM_STATE_SENDLIST_SENDWPS;
					wpm.current_wp_id = wpr.seq;
					wpm_send_waypoint(wpm.current_partner_sysid, wpm.current_partner_compid, wpr.seq);

				} else {
					// if (verbose)
					{
						if (!(wpm.current_state == WPM_STATE_SENDLIST || wpm.current_state == WPM_STATE_SENDLIST_SENDWPS)) {
#ifndef WPM_NO_PRINTF
							if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because i'm doing something else already (state=%i).\n", wpm.current_state);
#endif
                                                        missionlib_send_mavlink_gcs_string("REJ. WP CMD: Busy");
							break;

						} else if (wpm.current_state == WPM_STATE_SENDLIST) {
							if (wpr.seq != 0) {
#ifndef WPM_NO_PRINTF
								if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);
#endif
                                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: First id != 0");
							}

						} else if (wpm.current_state == WPM_STATE_SENDLIST_SENDWPS) {
							if (wpr.seq != wpm.current_wp_id && wpr.seq != wpm.current_wp_id + 1) {
#ifndef WPM_NO_PRINTF
								if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, wpm.current_wp_id, wpm.current_wp_id + 1);
#endif
                                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: Req. WP was unexpected");

							} else if (wpr.seq >= wpm.size) {
#ifndef WPM_NO_PRINTF
								if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);
#endif
                                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: Req. WP not in list");
							}

						} else {
#ifndef WPM_NO_PRINTF
							if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST - FIXME: missed error description\n");
#endif
                                                        missionlib_send_mavlink_gcs_string("REJ. WP CMD: ?");
						}
					}
				}

			} else {
				//we we're target but already communicating with someone else
				if ((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid)) {
#ifndef WPM_NO_PRINTF
					if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, wpm.current_partner_sysid);
#endif
                                        missionlib_send_mavlink_gcs_string("REJ. WP CMD: Busy");

				} else {
#ifndef WPM_NO_PRINTF
					if (WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
                                        missionlib_send_mavlink_gcs_string("REJ. WP CMD: target id mismatch");
				}

			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t wpc;
			mavlink_msg_mission_count_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm.timestamp_lastaction = now;

				if (wpm.current_state == WPM_STATE_IDLE || (wpm.current_state == WPM_STATE_GETLIST && wpm.current_wp_id == 0)) {
					if (wpc.count > 0) {
						if (wpm.current_state == WPM_STATE_IDLE) {
#ifndef WPM_NO_PRINTF
							if (WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) from %u changing state to WPM_STATE_GETLIST\n", wpc.count, msg->sysid);
#endif
                                                        missionlib_send_mavlink_gcs_string("WP CMD OK: state -> GETLIST");
						}

						if (wpm.current_state == WPM_STATE_GETLIST) {
#ifndef WPM_NO_PRINTF
							if (WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) again from %u\n", wpc.count, msg->sysid);
#endif
                                                        missionlib_send_mavlink_gcs_string("WP CMD OK AGAIN");
						}

						wpm.current_state = WPM_STATE_GETLIST;
						wpm.current_wp_id = 0;
						wpm.current_partner_sysid = msg->sysid;
						wpm.current_partner_compid = msg->compid;
						wpm.current_count = wpc.count;

#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("clearing receive buffer and readying for receiving waypoints\n");
#endif
                                                missionlib_send_mavlink_gcs_string("CLR RCV BUF: READY");
						wpm.rcv_size = 0;

						wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);

					} else if (wpc.count == 0) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("got waypoint count of 0, clearing waypoint list and staying in state WPM_STATE_IDLE\n");
#endif
                                                missionlib_send_mavlink_gcs_string("COUNT 0");
						wpm.rcv_size = 0;
						wpm.current_active_wp_id = -1;
						wpm.yaw_reached = false;
						wpm.pos_reached = false;
						break;

					} else {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Ignoring MAVLINK_MSG_ID_MISSION_ITEM_COUNT from %u with count of %u\n", msg->sysid, wpc.count);
#endif
                                                missionlib_send_mavlink_gcs_string("IGN WP CMD");
					}

				} else {
					if (!(wpm.current_state == WPM_STATE_IDLE || wpm.current_state == WPM_STATE_GETLIST)) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm doing something else already (state=%i).\n", wpm.current_state);
#endif
                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: Busy");

					} else if (wpm.current_state == WPM_STATE_GETLIST && wpm.current_wp_id != 0) {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm already receiving waypoint %u.\n", wpm.current_wp_id);
#endif
                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: Busy");

					} else {
#ifndef WPM_NO_PRINTF
						if (WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT - FIXME: missed error description\n");
#endif
                                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: ?");
					}
				}

			} else {
#ifndef WPM_NO_PRINTF
				if (WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");
#endif
                                missionlib_send_mavlink_gcs_string("REJ. WP CMD: target id mismatch");
			}

		}
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM: {
			mission_item_t wp;
			mavlink_mission_item_t mwp;

			mavlink_msg_mission_item_decode(msg, &mwp);

			wp.aceptable_radius = mwp.param1;
			wp.acceptable_time = mwp.param2;
			wp.orbit = mwp.param3;
			wp.yaw = mwp.param4;
			wp.lat = mwp.x;
			wp.lon = mwp.y;
			wp.alt = mwp.z;
			wp.seq = mwp.seq;
			wp.command = mwp.command;
			wp.frame = mwp.frame;
			wp.current = mwp.current;
			wp.autocontinue = mwp.autocontinue;

			missionlib_send_mavlink_gcs_string("GOT WP");

			if (mwp.target_system == mavlink_system.sysid && mwp.target_component == mavlink_wpm_comp_id) {

				wpm.timestamp_lastaction = now;

				//ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
				if ((wpm.current_state == WPM_STATE_GETLIST && wp.seq == 0) || (wpm.current_state == WPM_STATE_GETLIST_GETWPS && wp.seq == wpm.current_wp_id && wp.seq < wpm.current_count)) {

					wpm.current_state = WPM_STATE_GETLIST_GETWPS;
					set_waypoint(wp.seq, &wp);

					wpm.current_wp_id = wp.seq + 1;

					if (wpm.current_wp_id == wpm.current_count && wpm.current_state == WPM_STATE_GETLIST_GETWPS) {
						missionlib_send_mavlink_gcs_string("GOT ALL WPS");
						// if (verbose) // printf("Got all %u waypoints, changing state to WPM_STATE_IDLE\n", wpm.current_count);

						wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);

						if (wpm.current_active_wp_id > wpm.rcv_size - 1) {
							wpm.current_active_wp_id = wpm.rcv_size - 1;
						}

						wpm.size = wpm.current_count;

						//get the new current waypoint
						int i;

						for (i = 0; i < wpm.size; i++) {
							mission_item_t work_wp;
							get_waypoint(i, &work_wp);

							if (work_wp.current == 1) {
								wpm.current_active_wp_id = i;
								//// if (verbose) // printf("New current waypoint %u\n", current_active_wp_id);
								wpm.yaw_reached = false;
								wpm.pos_reached = false;
								wpm_send_waypoint_current(wpm.current_active_wp_id);
								wpm_send_setpoint(wpm.current_active_wp_id);
								wpm.timestamp_firstinside_orbit = 0;
								break;
							}
						}

						if (i == wpm.size) {
							wpm.current_active_wp_id = -1;
							wpm.yaw_reached = false;
							wpm.pos_reached = false;
							wpm.timestamp_firstinside_orbit = 0;
						}

						wpm.current_state = WPM_STATE_IDLE;

						unsigned info = wpm.size;

						if (mission_pub > 0) {
							orb_publish(ORB_ID(mission), mission_pub, &info);

						} else {
							mission_pub = orb_advertise(ORB_ID(mission), &info);
						}

					} else {
						wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);
					}

				} else {
					if (wpm.current_state == WPM_STATE_IDLE) {
						//we're done receiving waypoints, answer with ack.
						wpm_send_waypoint_ack(wpm.current_partner_sysid, wpm.current_partner_compid, 0);
						printf("Received MAVLINK_MSG_ID_MISSION_ITEM while state=WPM_STATE_IDLE, answered with WAYPOINT_ACK.\n");
					}

					// if (verbose)
					{
						if (!(wpm.current_state == WPM_STATE_GETLIST || wpm.current_state == WPM_STATE_GETLIST_GETWPS)) {
//							 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u because i'm doing something else already (state=%i).\n", wp.seq, wpm.current_state);
							break;

						} else if (wpm.current_state == WPM_STATE_GETLIST) {
							if (!(wp.seq == 0)) {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the first waypoint ID (%u) was not 0.\n", wp.seq);
							} else {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u - FIXME: missed error description\n", wp.seq);
							}
						} else if (wpm.current_state == WPM_STATE_GETLIST_GETWPS) {
							if (!(wp.seq == wpm.current_wp_id)) {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was not the expected %u.\n", wp.seq, wpm.current_wp_id);
								wpm_send_waypoint_request(wpm.current_partner_sysid, wpm.current_partner_compid, wpm.current_wp_id);

							} else if (!(wp.seq < wpm.current_count)) {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was out of bounds.\n", wp.seq);
							} else {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u - FIXME: missed error description\n", wp.seq);
							}
						} else {
//							 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u - FIXME: missed error description\n", wp.seq);
						}
					}
				}
			} else {
				//we we're target but already communicating with someone else
				if ((mwp.target_system == mavlink_system.sysid /*&& wp.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm.current_partner_sysid && msg->compid == wpm.current_partner_compid) && wpm.current_state != WPM_STATE_IDLE) {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, wpm.current_partner_sysid);
				} else if (mwp.target_system == mavlink_system.sysid /* && wp.target_component == mavlink_wpm_comp_id*/) {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
			mavlink_mission_clear_all_t wpca;
			mavlink_msg_mission_clear_all_decode(msg, &wpca);

			if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm.current_state == WPM_STATE_IDLE) {
				wpm.timestamp_lastaction = now;

				// if (verbose) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
				// Delete all waypoints
				wpm.size = 0;
				wpm.current_active_wp_id = -1;
				wpm.yaw_reached = false;
				wpm.pos_reached = false;
				dm_clear(DM_KEY_WAY_POINTS);

			} else if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm.current_state != WPM_STATE_IDLE) {
				// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, wpm.current_state);
			}

			break;
		}

	default: {
			break;
		}
	}

}

/**
 * Set special vehicle setpoint fields based on current mission item.
 *
 * @return true if the mission item could be interpreted
 * successfully, it return false on failure.
 */
static bool set_special_fields(float param1, float param2, float param3, float param4, uint16_t command,
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
void waypoints_current_waypoint_changed(uint16_t index, float param1,
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
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
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

		mission_item_t last_wp;

		while (last_setpoint_index >= 0) {
			get_waypoint(last_setpoint_index, &last_wp);

			if (last_wp.frame == (int)MAV_FRAME_GLOBAL &&
			    (last_wp.command == (int)MAV_CMD_NAV_WAYPOINT ||
			     last_wp.command == (int)MAV_CMD_NAV_LOITER_TURNS ||
			     last_wp.command == (int)MAV_CMD_NAV_LOITER_TIME ||
			     last_wp.command == (int)MAV_CMD_NAV_LOITER_UNLIM)) {
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
		if (wpm.size > 1) {
			next_setpoint_index = index + 1;
		}

		mission_item_t next_wp;

		while (next_setpoint_index < wpm.size - 1) {
			get_waypoint(next_setpoint_index, &next_wp);

			if (next_wp.frame == (int)MAV_FRAME_GLOBAL &&
			    (next_wp.command == MAV_CMD_NAV_WAYPOINT ||
			     next_wp.command == MAV_CMD_NAV_LOITER_TURNS ||
			     next_wp.command == MAV_CMD_NAV_LOITER_TIME ||
			     next_wp.command == MAV_CMD_NAV_LOITER_UNLIM)) {
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
			sp.lat = last_wp.lat * 1e7f;
			sp.lon = last_wp.lon * 1e7f;
			sp.altitude = last_wp.alt;
			sp.altitude_is_relative = false;
			sp.yaw = (last_wp.yaw / 180.0f) * M_PI_F - M_PI_F;
			set_special_fields(last_wp.aceptable_radius,
					   last_wp.acceptable_time,
					   last_wp.orbit,
					   last_wp.yaw,
					   last_wp.command, &sp);
			memcpy(&(triplet.previous), &sp, sizeof(sp));
		}

		if (next_setpoint_valid) {
			triplet.next_valid = true;
			struct vehicle_global_position_setpoint_s sp;
			sp.lat = next_wp.lat * 1e7f;
			sp.lon = next_wp.lon * 1e7f;
			sp.altitude = next_wp.alt;
			sp.altitude_is_relative = false;
			sp.yaw = (next_wp.yaw / 180.0f) * M_PI_F - M_PI_F;
			set_special_fields(next_wp.aceptable_radius,
					   next_wp.acceptable_time,
					   next_wp.orbit,
					   next_wp.yaw,
					   next_wp.command, &sp);
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
		missionlib_send_mavlink_gcs_string("[mp] Unknown waypoint type, ignoring.");
		return;
	}

	/* only set this for known waypoint types (non-navigation types would have returned earlier) */
	last_waypoint_index = index;

	missionlib_send_mavlink_gcs_string(buf);
	printf("%s\n", buf);
}
