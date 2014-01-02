/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
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
#include "waypoints.h"
#include "util.h"
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

bool verbose = true;

orb_advert_t mission_pub = -1;
struct mission_s mission;

uint8_t mavlink_wpm_comp_id = MAV_COMP_ID_MISSIONPLANNER;

void
mavlink_missionlib_send_message(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(missionlib_msg_buf, msg);

	mavlink_send_uart_bytes(chan, missionlib_msg_buf, len);
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
		mavlink_missionlib_send_message(&msg);
		return OK;

	} else {
		return 1;
	}
}

void publish_mission()
{
	/* Initialize mission publication if necessary */
	if (mission_pub < 0) {
		mission_pub = orb_advertise(ORB_ID(mission), &mission);

	} else {
		orb_publish(ORB_ID(mission), mission_pub, &mission);
	}
}

int map_mavlink_mission_item_to_mission_item(const mavlink_mission_item_t *mavlink_mission_item, struct mission_item_s *mission_item)
{
	/* only support global waypoints for now */
	switch (mavlink_mission_item->frame) {
		case MAV_FRAME_GLOBAL:
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
			mission_item->altitude = mavlink_mission_item->z;
			mission_item->altitude_is_relative = false;
			break;

		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
			mission_item->altitude = mavlink_mission_item->z;
			mission_item->altitude_is_relative = true;
			break;

		case MAV_FRAME_LOCAL_NED:
		case MAV_FRAME_LOCAL_ENU:
			return MAV_MISSION_UNSUPPORTED_FRAME;
		case MAV_FRAME_MISSION:
		default:
			return MAV_MISSION_ERROR;
	}

	switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_TAKEOFF:
			mission_item->pitch_min = mavlink_mission_item->param2;
			break;
		default:
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			break;
	}

	mission_item->yaw = _wrap_pi(mavlink_mission_item->param4*M_DEG_TO_RAD_F);
	mission_item->loiter_radius = fabsf(mavlink_mission_item->param3);
	mission_item->loiter_direction = (mavlink_mission_item->param3 > 0) ? 1 : -1; /* 1 if positive CW, -1 if negative CCW */
	mission_item->nav_cmd = mavlink_mission_item->command;

	mission_item->time_inside = mavlink_mission_item->param1 / 1e3f; /* from milliseconds to seconds */
	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;
	mission_item->origin = ORIGIN_MAVLINK;

	return OK;
}

int map_mission_item_to_mavlink_mission_item(const struct mission_item_s *mission_item, mavlink_mission_item_t *mavlink_mission_item)
{
	if (mission_item->altitude_is_relative) {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
	} else {
		mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	}
	
	switch (mission_item->nav_cmd) {
		case NAV_CMD_TAKEOFF:
			mavlink_mission_item->param2 = mission_item->pitch_min;
			break;
		default:
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			break;
	}

	mavlink_mission_item->x = (float)mission_item->lat;
	mavlink_mission_item->y = (float)mission_item->lon;
	mavlink_mission_item->z = mission_item->altitude;

	mavlink_mission_item->param4 = mission_item->yaw*M_RAD_TO_DEG_F;
	mavlink_mission_item->param3 = mission_item->loiter_radius*(float)mission_item->loiter_direction;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->param1 = mission_item->time_inside * 1e3f; /* from seconds to milliseconds */
	mavlink_mission_item->autocontinue = mission_item->autocontinue;
	// mavlink_mission_item->seq = mission_item->index;

	return OK;
}

void mavlink_wpm_init(mavlink_wpm_storage *state)
{
	state->size = 0;
	state->max_size = MAVLINK_WPM_MAX_WP_COUNT;
	state->current_state = MAVLINK_WPM_STATE_IDLE;
	state->current_partner_sysid = 0;
	state->current_partner_compid = 0;
	state->timestamp_lastaction = 0;
	state->timestamp_last_send_setpoint = 0;
	state->timeout = MAVLINK_WPM_PROTOCOL_TIMEOUT_DEFAULT;
	state->current_dataman_id = 0;
}

/*
 *  @brief Sends an waypoint ack message
 */
void mavlink_wpm_send_waypoint_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_message_t msg;
	mavlink_mission_ack_t wpa;

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;

	mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpa);
	mavlink_missionlib_send_message(&msg);

	if (verbose) warnx("Sent waypoint ack (%u) to ID %u", wpa.type, wpa.target_system);
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
		mavlink_message_t msg;
		mavlink_mission_current_t wpc;

		wpc.seq = seq;

		mavlink_msg_mission_current_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
		mavlink_missionlib_send_message(&msg);

		if (verbose) warnx("Broadcasted new current waypoint %u", wpc.seq);

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: wp index out of bounds");
		if (verbose) warnx("ERROR: index out of bounds");
	}
}

void mavlink_wpm_send_waypoint_count(uint8_t sysid, uint8_t compid, uint16_t count)
{
	mavlink_message_t msg;
	mavlink_mission_count_t wpc;

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = mission.count;

	mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpc);
	mavlink_missionlib_send_message(&msg);

	if (verbose) warnx("Sent waypoint count (%u) to ID %u", wpc.count, wpc.target_system);
}

void mavlink_wpm_send_waypoint(uint8_t sysid, uint8_t compid, uint16_t seq)
{

	struct mission_item_s mission_item;
	ssize_t len = sizeof(struct mission_item_s);
	
	dm_item_t dm_current;

	if (wpm->current_dataman_id == 0) {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_0;
	} else {
		dm_current = DM_KEY_WAYPOINTS_OFFBOARD_1;
	}

	if (dm_read(dm_current, seq, &mission_item, len) == len) {

		/* create mission_item_s from mavlink_mission_item_t */
		mavlink_mission_item_t wp;
		map_mission_item_to_mavlink_mission_item(&mission_item, &wp);

		mavlink_message_t msg;
		wp.target_system = sysid;
		wp.target_component = compid;
		wp.seq = seq;
		mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wp);
		mavlink_missionlib_send_message(&msg);

		if (verbose) warnx("Sent waypoint %u to ID %u", wp.seq, wp.target_system);
	} else {
		mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
		if (verbose) warnx("ERROR: could not read WP%u", seq);
	}
}

void mavlink_wpm_send_waypoint_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < wpm->max_size) {
		mavlink_message_t msg;
		mavlink_mission_request_t wpr;
		wpr.target_system = sysid;
		wpr.target_component = compid;
		wpr.seq = seq;
		mavlink_msg_mission_request_encode(mavlink_system.sysid, mavlink_wpm_comp_id, &msg, &wpr);
		mavlink_missionlib_send_message(&msg);

		if (verbose) warnx("Sent waypoint request %u to ID %u", wpr.seq, wpr.target_system);

	} else {
		mavlink_missionlib_send_gcs_string("ERROR: Waypoint index exceeds list capacity");
		if (verbose) warnx("ERROR: Waypoint index exceeds list capacity");
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

	if (verbose) warnx("Sent waypoint %u reached message", wp_reached.seq);
}


void mavlink_waypoint_eventloop(uint64_t now)
{
	/* check for timed-out operations */
	if (now - wpm->timestamp_lastaction > wpm->timeout && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

		mavlink_missionlib_send_gcs_string("Operation timeout");

		if (verbose) warnx("Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE", wpm->current_state);

		wpm->current_state = MAVLINK_WPM_STATE_IDLE;
		wpm->current_partner_sysid = 0;
		wpm->current_partner_compid = 0;
	}
}


void mavlink_wpm_message_handler(const mavlink_message_t *msg)
{
	uint64_t now = hrt_absolute_time();

	switch (msg->msgid) {

		case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
					if (wpm->current_wp_id == wpm->size - 1) {

						wpm->current_state = MAVLINK_WPM_STATE_IDLE;
						wpm->current_wp_id = 0;
					}
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
				if (verbose) warnx("REJ. WP CMD: curr partner id mismatch");
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

						mission.current_index = wpc.seq;
						publish_mission();
						
						mavlink_wpm_send_waypoint_current(wpc.seq);

					} else {
						mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Not in list");
						if (verbose) warnx("IGN WP CURR CMD: Not in list");
					}

				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Busy");
					if (verbose) warnx("IGN WP CURR CMD: Busy");

				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
				if (verbose) warnx("REJ. WP CMD: target id mismatch");
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
						
						wpm->current_state = MAVLINK_WPM_STATE_SENDLIST;
						wpm->current_wp_id = 0;
						wpm->current_partner_sysid = msg->sysid;
						wpm->current_partner_compid = msg->compid;

					} else {
						if (verbose) warnx("No waypoints send");
					}

					wpm->current_count = wpm->size;
					mavlink_wpm_send_waypoint_count(msg->sysid, msg->compid, wpm->current_count);

				} else {
					mavlink_missionlib_send_gcs_string("IGN REQUEST LIST: Busy");
					if (verbose) warnx("IGN REQUEST LIST: Busy");
				}
			} else {
				mavlink_missionlib_send_gcs_string("REJ. REQUEST LIST: target id mismatch");
				if (verbose) warnx("REJ. REQUEST LIST: target id mismatch");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_REQUEST: {
			mavlink_mission_request_t wpr;
			mavlink_msg_mission_request_decode(msg, &wpr);

			if (msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid && wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpr.seq >= wpm->size) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP not in list");
					if (verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was out of bounds.", wpr.seq);
					break;
				}

				/* 
				 * Ensure that we are in the correct state and that the first request has id 0 
				 * and the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
				 */
				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST) {

					if (wpr.seq == 0) {
						if (verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u changing state to MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);
						wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: First id != 0");
						if (verbose) warnx("REJ. WP CMD: First id != 0");
						break;
					}

				} else if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {

					if (wpr.seq == wpm->current_wp_id) {

						if (verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u (again) from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);

					} else if (wpr.seq == wpm->current_wp_id + 1) {

						if (verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_REQUEST of waypoint %u from %u staying in state MAVLINK_WPM_STATE_SENDLIST_SENDWPS", wpr.seq, msg->sysid);
					
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Req. WP was unexpected");
						if (verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because the requested waypoint ID (%u) was not the expected (%u or %u).", wpr.seq, wpm->current_wp_id, wpm->current_wp_id + 1);
						break;
					}

				} else {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
					if (verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST because i'm doing something else already (state=%i).", wpm->current_state);
					break;
				}

				wpm->current_wp_id = wpr.seq;
				wpm->current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;

				if (wpr.seq < wpm->size) {

					mavlink_wpm_send_waypoint(wpm->current_partner_sysid, wpm->current_partner_compid,wpm->current_wp_id);

				} else {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					if (verbose) warnx("ERROR: Waypoint %u out of bounds", wpr.seq);
				}


			} else {
				//we we're target but already communicating with someone else
				if ((wpr.target_system == mavlink_system.sysid /*&& wpr.target_component == mavlink_wpm_comp_id*/) && !(msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid)) {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
					if (verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_REQUEST from ID %u because i'm already talking to ID %u.", msg->sysid, wpm->current_partner_sysid);

				} else {

					mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
					if (verbose) warnx("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
				}
			}
			break;
		}

		case MAVLINK_MSG_ID_MISSION_COUNT: {
			mavlink_mission_count_t wpc;
			mavlink_msg_mission_count_decode(msg, &wpc);

			if (wpc.target_system == mavlink_system.sysid/* && wpc.target_component == mavlink_wpm_comp_id*/) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {

					if (wpc.count > NUM_MISSIONS_SUPPORTED) {
						if (verbose) warnx("Too many waypoints: %d, supported: %d", wpc.count, NUM_MISSIONS_SUPPORTED);
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_NO_SPACE);
						break;
					}

					if (wpc.count == 0) {
						mavlink_missionlib_send_gcs_string("COUNT 0");
						if (verbose) warnx("got waypoint count of 0, clearing waypoint list and staying in state MAVLINK_WPM_STATE_IDLE");
						break;
					}
					
					if (verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) from %u changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

					wpm->current_state = MAVLINK_WPM_STATE_GETLIST;
					wpm->current_wp_id = 0;
					wpm->current_partner_sysid = msg->sysid;
					wpm->current_partner_compid = msg->compid;
					wpm->current_count = wpc.count;

					mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);

				} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

					if (wpm->current_wp_id == 0) {
						mavlink_missionlib_send_gcs_string("WP CMD OK AGAIN");
						if (verbose) warnx("Got MAVLINK_MSG_ID_MISSION_ITEM_COUNT (%u) again from %u", wpc.count, msg->sysid);
					} else {
						mavlink_missionlib_send_gcs_string("REJ. WP CMD: Busy");
						if (verbose) warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM_COUNT because i'm already receiving waypoint %u.", wpm->current_wp_id);
					}
				} else {
						mavlink_missionlib_send_gcs_string("IGN MISSION_COUNT CMD: Busy");
						if (verbose) warnx("IGN MISSION_COUNT CMD: Busy");
				}
			} else {

				mavlink_missionlib_send_gcs_string("REJ. WP COUNT CMD: target id mismatch");
				if (verbose) warnx("IGNORED WAYPOINT COUNT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
			}
		}
		break;

		case MAVLINK_MSG_ID_MISSION_ITEM: {
			mavlink_mission_item_t wp;
			mavlink_msg_mission_item_decode(msg, &wp);

			if (wp.target_system == mavlink_system.sysid && wp.target_component == mavlink_wpm_comp_id) {

				wpm->timestamp_lastaction = now;

				/*
				 * ensure that we are in the correct state and that the first waypoint has id 0
				 * and the following waypoints have the correct ids
				 */

				if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST) {

				 	if (wp.seq != 0) {
				 		mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP not 0");
				 		warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the first waypoint ID (%u) was not 0.", wp.seq);
				 		break;
				 	}
				} else if (wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {

					if (wp.seq >= wpm->current_count) {
						mavlink_missionlib_send_gcs_string("Ignored MISSION_ITEM WP out of bounds");
						warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was out of bounds.", wp.seq);
						break;
					}

					if (wp.seq != wpm->current_wp_id) {
						warnx("Ignored MAVLINK_MSG_ID_MISSION_ITEM because the waypoint ID (%u) was not the expected %u.", wp.seq, wpm->current_wp_id);
						mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);
						break;
					}
				}

				wpm->current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;

				struct mission_item_s mission_item;

				int ret = map_mavlink_mission_item_to_mission_item(&wp, &mission_item);

				if (ret != OK) {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, ret);
					wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

				ssize_t len = sizeof(struct mission_item_s);

				dm_item_t dm_next;

				if (wpm->current_dataman_id == 0) {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_1;
					mission.dataman_id = 1;
				} else {
					dm_next = DM_KEY_WAYPOINTS_OFFBOARD_0;
					mission.dataman_id = 0;
				}

				if (dm_write(dm_next, wp.seq, DM_PERSIST_IN_FLIGHT_RESET, &mission_item, len) != len) {
					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					wpm->current_state = MAVLINK_WPM_STATE_IDLE;
					break;
				}

				if (wp.current) {
					mission.current_index = wp.seq;
				}

				wpm->current_wp_id = wp.seq + 1;

				if (wpm->current_wp_id == wpm->current_count && wpm->current_state == MAVLINK_WPM_STATE_GETLIST_GETWPS) {
					
					if (verbose) warnx("Got all %u waypoints, changing state to MAVLINK_WPM_STATE_IDLE", wpm->current_count);

					mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ACCEPTED);

					mission.count = wpm->current_count;
					
					publish_mission();

					wpm->current_dataman_id = mission.dataman_id;
					wpm->size = wpm->current_count;

					wpm->current_state = MAVLINK_WPM_STATE_IDLE;

				} else {
					mavlink_wpm_send_waypoint_request(wpm->current_partner_sysid, wpm->current_partner_compid, wpm->current_wp_id);
				}

			} else {
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
				if (verbose) warnx("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
			}

			break;
		}

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
			mavlink_mission_clear_all_t wpca;
			mavlink_msg_mission_clear_all_decode(msg, &wpca);

			if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */) {

				if (wpm->current_state == MAVLINK_WPM_STATE_IDLE) {
					wpm->timestamp_lastaction = now;

					wpm->size = 0;

					/* prepare mission topic */
					mission.dataman_id = -1;
					mission.count = 0;
					mission.current_index = -1;
					publish_mission();

					if (dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_0) == OK && dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1) == OK) {
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ACCEPTED);
					} else {
						mavlink_wpm_send_waypoint_ack(wpm->current_partner_sysid, wpm->current_partner_compid, MAV_MISSION_ERROR);
					}

					
				} else {
					mavlink_missionlib_send_gcs_string("IGN WP CLEAR CMD: Busy");
					if (verbose) warnx("IGN WP CLEAR CMD: Busy");
				}


			} else if (wpca.target_system == mavlink_system.sysid /*&& wpca.target_component == mavlink_wpm_comp_id */ && wpm->current_state != MAVLINK_WPM_STATE_IDLE) {

				mavlink_missionlib_send_gcs_string("REJ. WP CLERR CMD: target id mismatch");
				if (verbose) warnx("IGNORED WAYPOINT CLEAR COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH");
			}

			break;
		}

	default: {
			/* other messages might should get caught by mavlink and others */
			break;
		}
	}
}
