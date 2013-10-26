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

#include "mavlink_bridge_header.h"
#include "missionlib.h"
#include "waypoints.h"
#include "util.h"

#ifndef FM_PI
#define FM_PI 3.1415926535897932384626433832795f
#endif

bool debug = false;
bool verbose = false;


#define MAVLINK_WPM_NO_PRINTF

uint8_t mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;

void mavlink_wpm_init(mavlink_wpm_storage *state)
{
	// Set all waypoints to zero

	// Set count to zero
	state->size = 0;
	state->max_size = MAVLINK_WPM_MAX_WP_COUNT;
	state->current_state = MAVLINK_WPM_STATE_IDLE;
	state->current_partner_sysid = 0;
	state->current_partner_compid = 0;
	state->timestamp_lastaction = 0;
	state->timestamp_last_send_setpoint = 0;
	state->timeout = MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT;
	state->delay_setpoint = MAVLINK_WPM_SETPOINT_DELAY_DEFAULT;
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
void mavlink_wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_mission_ack_t wpa;

	wpa.target_system = wpm->current_partner_sysid;
	wpa.target_component = wpm->current_partner_compid;
	wpa.type = type;

	mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpa);
	mavlink_missionlib_send_message(&msg);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	if (MAVLINK_WPM_TEXT_FEEDBACK) {
#ifdef MAVLINK_WPM_NO_PRINTF
		mavlink_missionlib_send_gcs_string("Sent waypoint ACK");
#else

		if (MAVLINK_WPM_VERBOSE) printf("Sent waypoint ack (%u) to ID %u\n", wpa.type, wpa.target_system);

#endif
		mavlink_missionlib_send_gcs_string("Sent waypoint ACK");
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
void mavlink_wpm_send_waypoint_current(uint16_t seq)
{
	if (seq < wpm->size) {
		mavlink_mission_item_t *cur = &(wpm->waypoints[seq]);

		mavlink_message_t msg;
		mavlink_mission_current_t wpc;

		wpc.seq = cur->seq;

		mavlink_msg_mission_current_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
		mavlink_missionlib_send_message(&msg);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("Set current waypoint\n"); //// printf("Broadcasted new current waypoint %u\n", wpc.seq);

	} else {
		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("ERROR: wp index out of bounds\n");
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
void mavlink_wpm_send_setpoint(uint16_t seq)
{
	if (seq < wpm->size) {
		mavlink_mission_item_t *cur = &(wpm->waypoints[seq]);
		mavlink_missionlib_current_waypoint_changed(cur->seq, cur->param1,
				cur->param2, cur->param3, cur->param4, cur->x,
				cur->y, cur->z, cur->frame, cur->command);

		wpm->timestamp_last_send_setpoint = mavlink_missionlib_get_system_timestamp();

	} else {
		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("ERROR: Waypoint index out of bounds\n"); //// if (verbose) // printf("ERROR: index out of bounds\n");
	}
}

void mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_mission_count_t wpc;

	wpc.target_system = wpm->current_partner_sysid;
	wpc.target_component = wpm->current_partner_compid;
	wpc.count = count;

	mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
	mavlink_missionlib_send_message(&msg);

	if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("Sent waypoint count"); //// if (verbose) // printf("Sent waypoint count (%u) to ID %u\n", wpc.count, wpc.target_system);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

void mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm->size) {
		mavlink_message_t msg;
		mavlink_mission_item_t *wp = &(wpm->waypoints[seq]);
		wp->target_system = wpm->current_partner_sysid;
		wp->target_component = wpm->current_partner_compid;
		mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, wp);
		mavlink_missionlib_send_message(&msg);

		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("Sent waypoint"); //// if (verbose) // printf("Sent waypoint %u to ID %u\n", wp->seq, wp->target_system);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	} else {
		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("ERROR: Waypoint index out of bounds\n");
	}
}

void mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm->max_size) {
		mavlink_message_t msg;
		mavlink_mission_request_t wpr;
		wpr.target_system = wpm->current_partner_sysid;
		wpr.target_component = wpm->current_partner_compid;
		wpr.seq = seq;
		mavlink_msg_mission_request_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpr);
		mavlink_missionlib_send_message(&msg);

		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("Sent waypoint request"); //// if (verbose) // printf("Sent waypoint request %u to ID %u\n", wpr.seq, wpr.target_system);

		// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));

	} else {
		if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("ERROR: Waypoint index exceeds list capacity\n");
	}
}

/*
 *  @brief emits a message that a waypoint reached
 *
 *  This function broadcasts a message that a waypoint is reached.
 *
 *  @param seq The waypoint sequence number the MAV has reached.
 */
void mavlink_wpm_send_waypoint_reached(uint16_t seq)
{
	mavlink_message_t msg;
	mavlink_mission_item_reached_t wp_reached;

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wp_reached);
	mavlink_missionlib_send_message(&msg);

	if (MAVLINK_WPM_TEXT_FEEDBACK) mavlink_missionlib_send_gcs_string("Sent waypoint reached message"); //// if (verbose) // printf("Sent waypoint %u reached message\n", wp_reached.seq);

	// FIXME TIMING usleep(paramClient->getParamValue("PROTOCOLDELAY"));
}

/*
 * Calculate distance in global frame.
 *
 * The distance calculation is based on the WGS84 geoid (GPS)
 */
float mavlink_wpm_distance_to_point_global_wgs84(uint16_t seq, float lat, float lon, float alt, float *dist_xy, float *dist_z)
{

	if (seq < wpm->size) {
		mavlink_mission_item_t *wp = &(wpm->waypoints[seq]);

		double current_x_rad = wp->x / 180.0 * M_PI;
		double current_y_rad = wp->y / 180.0 * M_PI;
		double x_rad = lat / 180.0 * M_PI;
		double y_rad = lon / 180.0 * M_PI;

		double d_lat = x_rad - current_x_rad;
		double d_lon = y_rad - current_y_rad;

		double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0f) * cos(current_x_rad) * cos(x_rad);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));

		const double radius_earth = 6371000.0;

		float dxy = radius_earth * c;
		float dz = alt - wp->z;

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
float mavlink_wpm_distance_to_point_local(uint16_t seq, float x, float y, float z, float *dist_xy, float *dist_z)
{
	if (seq < wpm->size) {
		mavlink_mission_item_t *cur = &(wpm->waypoints[seq]);

		float dx = (cur->x - x);
		float dy = (cur->y - y);
		float dz = (cur->z - z);

		*dist_xy = sqrtf(dx * dx + dy * dy);
		*dist_z = fabsf(dz);

		return sqrtf(dx * dx + dy * dy + dz * dz);

	} else {
		return -1.0f;
	}
}

void check_waypoints_reached(uint64_t now, const struct vehicle_global_position_s *global_pos, struct vehicle_local_position_s *local_pos, float turn_distance)
{
	static uint16_t counter;

	if ((!global_pos->valid && !local_pos->xy_valid) ||
		/* no waypoint */
		wpm->size == 0) {
		/* nothing to check here, return */
		return;
	}

	if (wpm->current_active_wp_id < wpm->size) {

		float orbit;
		if (wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_WAYPOINT) {

			orbit = wpm->waypoints[wpm->current_active_wp_id].param2;

		} else if (wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TURNS ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TIME ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_UNLIM) {

			orbit = wpm->waypoints[wpm->current_active_wp_id].param3;
		} else {

			// XXX set default orbit via param
			orbit = 15.0f;
		}

		/* keep vertical orbit */
		float vertical_switch_distance = orbit;

		/* Take the larger turn distance - orbit or turn_distance */
		if (orbit < turn_distance)
			orbit = turn_distance;

		int coordinate_frame = wpm->waypoints[wpm->current_active_wp_id].frame;
		float dist = -1.0f;

		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		if (coordinate_frame == (int)MAV_FRAME_GLOBAL) {
			dist = mavlink_wpm_distance_to_point_global_wgs84(wpm->current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->alt, &dist_xy, &dist_z);

		} else if (coordinate_frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
			dist = mavlink_wpm_distance_to_point_global_wgs84(wpm->current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->relative_alt, &dist_xy, &dist_z);

		} else if (coordinate_frame == (int)MAV_FRAME_LOCAL_ENU || coordinate_frame == (int)MAV_FRAME_LOCAL_NED) {
			dist = mavlink_wpm_distance_to_point_local(wpm->current_active_wp_id, local_pos->x, local_pos->y, local_pos->z, &dist_xy, &dist_z);

		} else if (coordinate_frame == (int)MAV_FRAME_MISSION) {
			/* Check if conditions of mission item are satisfied */
			// XXX TODO
		}

		if (dist >= 0.f && dist_xy <= orbit && dist_z >= 0.0f && dist_z <= vertical_switch_distance) {
			wpm->pos_reached = true;
		}

		// check if required yaw reached
		float yaw_sp = _wrap_pi(wpm->waypoints[wpm->current_active_wp_id].param4 / 180.0f * FM_PI);
		float yaw_err = _wrap_pi(yaw_sp - local_pos->yaw);
		if (fabsf(yaw_err) < 0.05f) {
			wpm->yaw_reached = true;
		}
	}

	//check if the current waypoint was reached
	if (wpm->pos_reached && /*wpm->yaw_reached &&*/ !wpm->idle) {
		if (wpm->current_active_wp_id < wpm->size) {
			mavlink_mission_item_t *cur_wp = &(wpm->waypoints[wpm->current_active_wp_id]);

			if (wpm->timestamp_firstinside_orbit == 0) {
				// Announce that last waypoint was reached
				mavlink_wpm_send_waypoint_reached(cur_wp->seq);
				wpm->timestamp_firstinside_orbit = now;
			}

			// check if the MAV was long enough inside the waypoint orbit
			//if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))

			bool time_elapsed = false;

			if (now - wpm->timestamp_firstinside_orbit >= cur_wp->param1 * 1000 * 1000) {
				time_elapsed = true;
			} else if (cur_wp->command == (int)MAV_CMD_NAV_TAKEOFF) {
				time_elapsed = true;
			}

			if (time_elapsed) {

				/* safeguard against invalid missions with last wp autocontinue on */
				if (wpm->current_active_wp_id == wpm->size - 1) {
					/* stop handling missions here */
					cur_wp->autocontinue = false;
				}

				if (cur_wp->autocontinue) {

					cur_wp->current = 0;

					float navigation_lat = -1.0f;
					float navigation_lon = -1.0f;
					float navigation_alt = -1.0f;
					int navigation_frame = -1;

					/* initialize to current position in case we don't find a suitable navigation waypoint */
					if (global_pos->valid) {
						navigation_lat = global_pos->lat/1e7;
						navigation_lon = global_pos->lon/1e7;
						navigation_alt = global_pos->alt;
						navigation_frame = MAV_FRAME_GLOBAL;
					} else if (local_pos->xy_valid && local_pos->z_valid) {
						navigation_lat = local_pos->x;
						navigation_lon = local_pos->y;
						navigation_alt = local_pos->z;
						navigation_frame = MAV_FRAME_LOCAL_NED;
					}

					/* guard against missions without final land waypoint */
					/* only accept supported navigation waypoints, skip unknown ones */
					do {

						/* pick up the last valid navigation waypoint, this will be one we hold on to after the mission */
						if (wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_WAYPOINT ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TURNS ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TIME ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_UNLIM ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_TAKEOFF) {

							/* this is a navigation waypoint */
							navigation_frame = cur_wp->frame;
							navigation_lat = cur_wp->x;
							navigation_lon = cur_wp->y;
							navigation_alt = cur_wp->z;
						}

						if (wpm->current_active_wp_id == wpm->size - 1) {

							/* if we're not landing at the last nav waypoint, we're falling back to loiter */
							if (wpm->waypoints[wpm->current_active_wp_id].command != (int)MAV_CMD_NAV_LAND) {
								/* the last waypoint was reached, if auto continue is
								 * activated AND it is NOT a land waypoint, keep the system loitering there.
								 */
								cur_wp->command = MAV_CMD_NAV_LOITER_UNLIM;
								cur_wp->param3 = 20.0f; // XXX magic number 20 m loiter radius
								cur_wp->frame = navigation_frame;
								cur_wp->x = navigation_lat;
								cur_wp->y = navigation_lon;
								cur_wp->z = navigation_alt;
							}

							/* we risk an endless loop for missions without navigation waypoints, abort. */
							break;

						} else {
							if ((uint16_t)(wpm->current_active_wp_id + 1) < wpm->size)
								wpm->current_active_wp_id++;
						}
			
					} while (!(wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_WAYPOINT ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TURNS ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_TIME ||
				wpm->waypoints[wpm->current_active_wp_id].command == (int)MAV_CMD_NAV_LOITER_UNLIM));

					// Fly to next waypoint
					wpm->timestamp_firstinside_orbit = 0;
					mavlink_wpm_send_waypoint_current(wpm->current_active_wp_id);
					mavlink_wpm_send_setpoint(wpm->current_active_wp_id);
					wpm->waypoints[wpm->current_active_wp_id].current = true;
					wpm->pos_reached = false;
					wpm->yaw_reached = false;
					printf("Set new waypoint (%u)\n", wpm->current_active_wp_id);
				}
			}
		}

	} else {
		wpm->timestamp_lastoutside_orbit = now;
	}

	counter++;
}


int mavlink_waypoint_eventloop(uint64_t now, const struct vehicle_global_position_s *global_position, struct vehicle_local_position_s *local_position, struct navigation_capabilities_s *nav_cap)
{
	/* check for timed-out operations */
	if (now - wpm->timestamp_lastaction > wpm->timeout && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

#ifdef MAVLINK_WPM_NO_PRINTF
		mavlink_missionlib_send_gcs_string("Operation timeout switching -> IDLE");
#else

		if (MAVLINK_WPM_VERBOSE) printf("Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm->current_state);

#endif
		wpm->current_state = MAVLINK_WPM_STATE_IDLE;
		wpm->current_count = 0;
		wpm->current_partner_sysid = 0;
		wpm->current_partner_compid = 0;
		wpm->current_wp_id = -1;

		if (wpm->size == 0) {
			wpm->current_active_wp_id = -1;
		}
	}

	check_waypoints_reached(now, global_position, local_position, nav_cap->turn_distance);

	return OK;
}


void mavlink_wpm_message_handler(const mavlink_message_t *msg, const struct vehicle_global_position_s *global_pos , struct vehicle_local_position_s *local_pos)
{
	uint64_t now = mavlink_missionlib_get_system_timestamp();

	switch (msg->msgid) {

	case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
					if (wpm->current_wp_id == wpm->size - 1) {

						mavlink_missionlib_send_gcs_string("Got last WP ACK state -> IDLE");

						wpm->current_state = MAVLINK_WPM_STATE_IDLE;
						wpm->current_wp_id = 0;
					}
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
			mavlink_mission_set_current_t wpc;
			mavlink_msg_mission_set_current_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid /*&& wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					if (wpc.seq < wpm->size) {
						// if (verbose) // printf("Received MAVLINK_MSG_ID_MISSION_ITEM_SET_CURRENT\n");
						wpm->current_active_wp_id = wpc.seq;
						uint32_t i;

						for (i = 0; i < wpm->size; i++) {
							if (i == wpm->current_active_wp_id) {
								wpm->waypoints[i].current = true;

							} else {
								wpm->waypoints[i].current = false;
							}
						}

						mavlink_missionlib_send_gcs_string("NEW WP SET");

						wpm->yaw_reached = false;
						wpm->pos_reached = false;
						mavlink_wpm_send_waypoint_current(wpm->current_active_wp_id);
						mavlink_wpm_send_setpoint(wpm->current_active_wp_id);
						wpm->timestamp_firstinside_orbit = 0;

					} else {
						mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Not in list");
					}

				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Busy");

				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
			mavlink_mission_request_list_t wprl;
			mavlink_msg_mission_request_list_decode(msg, &wprl);

			if (wprl.target_system == mavlink_system.sysid /*&& wprl.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {
					if (wpm->size > 0) {
						//if (verbose && wpm->current_state == MAVLINK_WPM_STATE_IDLE) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST from %u changing state to MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
//                        if (verbose && wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST again from %u staying in state MAVLINK_WPM_STATE_SENDLIST\n", msg->sysid);
						wpm->current_state = MAVLINK_WPM_STATE_SENDLIST;
						wpm->current_wp_id = 0;
						wpm->current_partner_sysid = msg->sysid;
						wpm->current_partner_compid = msg->compid;

					} else {
						// if (verbose) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST from %u but have no waypoints, staying in \n", msg->sysid);
					}

					wpm->current_count = wpm->size;
					mavlink_wpm_send_waypoint_count(msg->sysid, msg->compid, wpm->current_count);

				} else {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST_LIST because i'm doing something else already (state=%i).\n", wpm->current_state);
				}
			} else {
				// if (verbose) // printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT MISMATCH\n");
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_REQUEST: {
			mavlink_mission_request_t wpr;
			mavlink_msg_mission_request_decode(msg, &wpr);

			if (msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				//ensure that we are in the correct state and that the first request has id 0 and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				if ((wpm->current_state == MAVLINK_WPM_STATE_SENDLIST && wpr.seq == 0) || (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && (wpr.seq == wpm->current_wp_id || wpr.seq == wpm->current_wp_id + 1) && wpr.seq < wpm->size)) {
					if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("GOT WP REQ, state -> SEND");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u changing state to MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

#endif
					}

					if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm->current_wp_id + 1) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("GOT 2nd WP REQ");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

#endif
					}

					if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS && wpr.seq == wpm->current_wp_id) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("GOT 2nd WP REQ");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u (again) from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS\n", wpr.seq, msg->sysid);

#endif
					}

					wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
					wpm->current_wp_id = wpr.seq;
					mavlink_wpm_send_waypoint(wpm->current_partner_sysid, wpm->current_partner_compid, wpr.seq);

				} else {
					// if (verbose)
					{
						if (!(wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS)) {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
#else

							if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because i'm doing something else already (state=%i).\n", wpm->current_state);

#endif
							break;

						} else if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {
							if (wpr.seq != 0) {
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_missionlib_send_gcs_string("REJ. WP CMD: First id != 0");
#else

								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the first requested waypoint ID (%u) was not 0.\n", wpr.seq);

#endif
							}

						} else if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
							if (wpr.seq != wpm->current_wp_id && wpr.seq != wpm->current_wp_id + 1) {
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP was unexpected");
#else

								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).\n", wpr.seq, wpm->current_wp_id, wpm->current_wp_id + 1);

#endif

							} else if (wpr.seq >= wpm->size) {
#ifdef MAVLINK_WPM_NO_PRINTF
								mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP not in list");
#else

								if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was out of bounds.\n", wpr.seq);

#endif
							}

						} else {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_missionlib_send_gcs_string("REJ. WP CMD: ?");
#else

							if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST - FIXME: missed error description\n");

#endif
						}
					}
				}

			} else {
				//we we're target but already communicating with someone else
				if ((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid)) {
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
#else

					if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST from ID %u because i'm already talking to ID %u.\n", msg->sysid, wpm->current_partner_sysid);

#endif

				} else {
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
#else

					if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");

#endif
				}

			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t wpc;
			mavlink_msg_mission_count_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE || (wpm->current_state == MAVLINK_WPM_STATE_GETLIST && wpm->current_wp_id == 0)) {
//                	printf("wpc count in: %d\n",wpc.count);
//                	printf("Comp id: %d\n",msg->compid);
//                	printf("Current partner sysid: %d\n",wpm->current_partner_sysid);

					if (wpc.count > 0) {
						if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_missionlib_send_gcs_string("WP CMD OK: state -> GETLIST");
#else

							if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) from %u changing state to MAVLINK_WPM_STATE_GETLIST\n", wpc.count, msg->sysid);

#endif
						}

						if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {
#ifdef MAVLINK_WPM_NO_PRINTF
							mavlink_missionlib_send_gcs_string("WP CMD OK AGAIN");
#else

							if (MAVLINK_WPM_VERBOSE) printf("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) again from %u\n", wpc.count, msg->sysid);

#endif
						}

						wpm->current_state = MAVLINK_WPM_STATE_GETLIST;
						wpm->current_wp_id = 0;
						wpm->current_partner_sysid = msg->sysid;
						wpm->current_partner_compid = msg->compid;
						wpm->current_count = wpc.count;

#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("CLR RCV BUF: READY");
#else

						if (MAVLINK_WPM_VERBOSE) printf("clearing receive buffer and readying for receiving waypoints\n");

#endif
						wpm->rcv_size = 0;
						//while(waypoints_receive_buffer->size() > 0)
//                        {
//                            delete waypoints_receive_buffer->back();
//                            waypoints_receive_buffer->pop_back();
//                        }

						mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);

					} else if (wpc.count == 0) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("COUNT 0");
#else

						if (MAVLINK_WPM_VERBOSE) printf("got waypoint count of 0, clearing waypoint list and staying in state MAVLINK_WPM_STATE_IDLE\n");

#endif
						wpm->rcv_size = 0;
						//while(waypoints_receive_buffer->size() > 0)
//                        {
//                            delete waypoints->back();
//                            waypoints->pop_back();
//                        }
						wpm->current_active_wp_id = -1;
						wpm->yaw_reached = false;
						wpm->pos_reached = false;
						break;

					} else {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("IGN WP CMD");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Ignoring MAVLINK_MSG_ID_MISSION_ITEM_COUNT from %u with count of %u\n", msg->sysid, wpc.count);

#endif
					}

				} else {
					if (!(wpm->current_state == MAVLINK_WPM_STATE_IDLE || wpm->current_state == MAVLINK_WPM_STATE_GETLIST)) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm doing something else already (state=%i).\n", wpm->current_state);

#endif

					} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST && wpm->current_wp_id != 0) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm already receiving waypoint %u.\n", wpm->current_wp_id);

#endif

					} else {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: ?");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT - FIXME: missed error description\n");

#endif
					}
				}

			} else {
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
#else

				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");

#endif
			}

		}
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM: {
			mavlink_mission_item_t wp;
			mavlink_msg_mission_item_decode(msg, &wp);

			mavlink_missionlib_send_gcs_string("GOT WP");
//			printf("sysid=%d, current_partner_sysid=%d\n", msg->sysid, wpm->current_partner_sysid);
//			printf("compid=%d, current_partner_compid=%d\n", msg->compid, wpm->current_partner_compid);

//            if((msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && (wp.target_system == mavlink_system.sysid /*&& wp.target_component == mavlink_wpm_comp_id*/))
			if (wp.target_system == mavlink_system.sysid && wp.target_component == mavlink_wpm_comp_id) {

				wpm->timestamp_lastaction = now;

//                printf("wpm->current_state=%u, wp.seq = %d, wpm->current_wp_id=%d\n", wpm->current_state, wp.seq, wpm->current_wp_id);

//                wpm->current_state = MAVLINK_WPM_STATE_GETLIST;//removeme debug XXX TODO

				//ensure that we are in the correct state and that the first waypoint has id 0 and the following waypoints have the correct ids
				if ((wpm->current_state == MAVLINK_WPM_STATE_GETLIST && wp.seq == 0) || (wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm->current_wp_id && wp.seq < wpm->current_count)) {
					//mavlink_missionlib_send_gcs_string("DEBUG 2");

//                    if (verbose && wpm->current_state == MAVLINK_WPM_STATE_GETLIST) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM %u from %u changing state to MAVLINK_WPM_STATE_GETLIST_GETWPS\n", wp.seq, msg->sysid);
//                    if (verbose && wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq == wpm->current_wp_id) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM %u from %u\n", wp.seq, msg->sysid);
//                    if (verbose && wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS && wp.seq-1 == wpm->current_wp_id) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM %u (again) from %u\n", wp.seq, msg->sysid);
//
					wpm->current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;
					mavlink_mission_item_t *newwp = &(wpm->rcv_waypoints[wp.seq]);
					memcpy(newwp, &wp, sizeof(mavlink_mission_item_t));
//                    printf("WP seq: %d\n",wp.seq);
					wpm->current_wp_id = wp.seq + 1;

					// if (verbose) // printf ("Added new waypoint to list. X= %f\t Y= %f\t Z= %f\t Yaw= %f\n", newwp->x, newwp->y, newwp->z, newwp->param4);
//					printf ("Added new waypoint to list. X= %f\t Y= %f\t Z= %f\t Yaw= %f\n", newwp->x, newwp->y, newwp->z, newwp->param4);

//					printf ("wpm->current_wp_id =%d, wpm->current_count=%d\n\n", wpm->current_wp_id, wpm->current_count);
					if (wpm->current_wp_id == wpm->current_count && wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {
						mavlink_missionlib_send_gcs_string("GOT ALL WPS");
						// if (verbose) // printf("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm->current_count);

						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, 0);

						if (wpm->current_active_wp_id > wpm->rcv_size - 1) {
							wpm->current_active_wp_id = wpm->rcv_size - 1;
						}

						// switch the waypoints list
						// FIXME CHECK!!!
						uint32_t i;

						for (i = 0; i < wpm->current_count; ++i) {
							wpm->waypoints[i] = wpm->rcv_waypoints[i];
						}

						wpm->size = wpm->current_count;

						//get the new current waypoint

						for (i = 0; i < wpm->size; i++) {
							if (wpm->waypoints[i].current == 1) {
								wpm->current_active_wp_id = i;
								//// if (verbose) // printf("New current waypoint %u\n", current_active_wp_id);
								wpm->yaw_reached = false;
								wpm->pos_reached = false;
								mavlink_wpm_send_waypoint_current(wpm->current_active_wp_id);
								mavlink_wpm_send_setpoint(wpm->current_active_wp_id);
								wpm->timestamp_firstinside_orbit = 0;
								break;
							}
						}

						if (i == wpm->size) {
							wpm->current_active_wp_id = -1;
							wpm->yaw_reached = false;
							wpm->pos_reached = false;
							wpm->timestamp_firstinside_orbit = 0;
						}

						wpm->current_state = MAVLINK_WPM_STATE_IDLE;

					} else {
						mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);
					}

				} else {
					if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
						//we're done receiving waypoints, answer with ack.
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, 0);
						printf("Received MAVLINK_MSG_ID_MISSION_ITEM while state=MAVLINK_WPM_STATE_IDLE, answered with WAYPOINT_ACK.\n");
					}

					// if (verbose)
					{
						if (!(wpm->current_state == MAVLINK_WPM_STATE_GETLIST || wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS)) {
//							 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u because i'm doing something else already (state=%i).\n", wp.seq, wpm->current_state);
							break;

						} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {
							if (!(wp.seq == 0)) {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the first waypoint ID (%u) was not 0.\n", wp.seq);
							} else {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u - FIXME: missed error description\n", wp.seq);
							}
						} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {
							if (!(wp.seq == wpm->current_wp_id)) {
//								 printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was not the expected %u.\n", wp.seq, wpm->current_wp_id);
								mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);

							} else if (!(wp.seq < wpm->current_count)) {
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
				if ((wp.target_system == mavlink_system.sysid /*&& wp.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u from ID %u because i'm already talking to ID %u.\n", wp.seq, msg->sysid, wpm->current_partner_sysid);
				} else if (wp.target_system == mavlink_system.sysid /* && wp.target_component == mavlink_wpm_comp_id*/) {
					// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM %u from ID %u because i have no idea what to do with it\n", wp.seq, msg->sysid);
				}
			}

			break;
		}

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
			mavlink_mission_clear_all_t wpca;
			mavlink_msg_mission_clear_all_decode(msg, &wpca);

			if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
				wpm->timestamp_lastaction = now;

				// if (verbose) // printf("Got MAVLINK_MSG_ID_MISSION_ITEM_CLEAR_LIST from %u deleting all waypoints\n", msg->sysid);
				// Delete all waypoints
				wpm->size = 0;
				wpm->current_active_wp_id = -1;
				wpm->yaw_reached = false;
				wpm->pos_reached = false;

			} else if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {
				// if (verbose) // printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_CLEAR_LIST from %u because i'm doing something else already (state=%i).\n", msg->sysid, wpm->current_state);
			}

			break;
		}

	default: {
			// if (debug) // printf("Waypoint: received message of unknown type");
			break;
		}
	}

	// check_waypoints_reached(now, global_pos, local_pos);
}
