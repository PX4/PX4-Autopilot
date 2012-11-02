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

//float mavlink_wpm_distance_to_segment(uint16_t seq, float x, float y, float z)
//{
//    if (seq < wpm->size)
//    {
//        mavlink_mission_item_t *cur = waypoints->at(seq);
//
//        const PxVector3 A(cur->x, cur->y, cur->z);
//        const PxVector3 C(x, y, z);
//
//        // seq not the second last waypoint
//        if ((uint16_t)(seq+1) < wpm->size)
//        {
//            mavlink_mission_item_t *next = waypoints->at(seq+1);
//            const PxVector3 B(next->x, next->y, next->z);
//            const float r = (B-A).dot(C-A) / (B-A).lengthSquared();
//            if (r >= 0 && r <= 1)
//            {
//                const PxVector3 P(A + r*(B-A));
//                return (P-C).length();
//            }
//            else if (r < 0.f)
//            {
//                return (C-A).length();
//            }
//            else
//            {
//                return (C-B).length();
//            }
//        }
//        else
//        {
//            return (C-A).length();
//        }
//    }
//    else
//    {
//        // if (verbose) // printf("ERROR: index out of bounds\n");
//    }
//    return -1.f;
//}

/*
 * Calculate distance in global frame.
 *
 * The distance calculation is based on the WGS84 geoid (GPS)
 */
float mavlink_wpm_distance_to_point_global_wgs84(uint16_t seq, float lat, float lon, float alt)
{
	//TODO: implement for z once altidude contoller is implemented

	static uint16_t counter;

//	if(counter % 10 == 0) printf(" x = %.10f, y = %.10f\n", x, y);
	if (seq < wpm->size) {
		mavlink_mission_item_t *cur = &(wpm->waypoints[seq]);

		double current_x_rad = cur->x / 180.0 * M_PI;
		double current_y_rad = cur->y / 180.0 * M_PI;
		double x_rad = lat / 180.0 * M_PI;
		double y_rad = lon / 180.0 * M_PI;

		double d_lat = x_rad - current_x_rad;
		double d_lon = y_rad - current_y_rad;

		double a = sin(d_lat / 2.0) * sin(d_lat / 2.0) + sin(d_lon / 2.0) * sin(d_lon / 2.0f) * cos(current_x_rad) * cos(x_rad);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));

		const double radius_earth = 6371000.0;

		return radius_earth * c;

	} else {
		return -1.0f;
	}

	counter++;

}

/*
 * Calculate distance in local frame (NED)
 */
float mavlink_wpm_distance_to_point_local(uint16_t seq, float x, float y, float z)
{
	if (seq < wpm->size) {
		mavlink_mission_item_t *cur = &(wpm->waypoints[seq]);

		float dx = (cur->x - x);
		float dy = (cur->y - y);
		float dz = (cur->z - z);

		return sqrt(dx * dx + dy * dy + dz * dz);

	} else {
		return -1.0f;
	}
}

void check_waypoints_reached(uint64_t now, const struct vehicle_global_position_s *global_pos, struct vehicle_local_position_s *local_pos)
{
	static uint16_t counter;

	// Do not flood the precious wireless link with debug data
	// if (wpm->size > 0 && counter % 10 == 0) {
	// 	printf("Currect active waypoint id: %i\n", wpm->current_active_wp_id);
	// }


	if (wpm->current_active_wp_id < wpm->size) {

		float orbit = wpm->waypoints[wpm->current_active_wp_id].param2;
		int coordinate_frame = wpm->waypoints[wpm->current_active_wp_id].frame;
		float dist = -1.0f;

		if (coordinate_frame == (int)MAV_FRAME_GLOBAL) {
			dist = mavlink_wpm_distance_to_point_global_wgs84(wpm->current_active_wp_id, global_pos->lat, global_pos->lon, global_pos->alt);

		} else if (coordinate_frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
			dist = mavlink_wpm_distance_to_point_global_wgs84(wpm->current_active_wp_id, global_pos->lat, global_pos->lon, global_pos->relative_alt);

		} else if (coordinate_frame == (int)MAV_FRAME_LOCAL_ENU || coordinate_frame == (int)MAV_FRAME_LOCAL_NED) {
			dist = mavlink_wpm_distance_to_point_local(wpm->current_active_wp_id, local_pos->x, local_pos->y, local_pos->z);

		} else if (coordinate_frame == (int)MAV_FRAME_MISSION) {
			/* Check if conditions of mission item are satisfied */
			// XXX TODO
		}

		if (dist >= 0.f && dist <= orbit /*&& wpm->yaw_reached*/) { //TODO implement yaw
			wpm->pos_reached = true;

			if (counter % 10 == 0)
				printf("Setpoint reached: %0.4f, orbit: %.4f\n", dist, orbit);
		}

//		else
//		{
//			if(counter % 100 == 0)
//				printf("Setpoint not reached yet: %0.4f, orbit: %.4f\n",dist,orbit);
//		}
	}

	//check if the current waypoint was reached
	if (wpm->pos_reached /*wpm->yaw_reached &&*/ && !wpm->idle) {
		if (wpm->current_active_wp_id < wpm->size) {
			mavlink_mission_item_t *cur_wp = &(wpm->waypoints[wpm->current_active_wp_id]);

			if (wpm->timestamp_firstinside_orbit == 0) {
				// Announce that last waypoint was reached
				printf("Reached waypoint %u for the first time \n", cur_wp->seq);
				mavlink_wpm_send_waypoint_reached(cur_wp->seq);
				wpm->timestamp_firstinside_orbit = now;
			}

			// check if the MAV was long enough inside the waypoint orbit
			//if (now-timestamp_lastoutside_orbit > (cur_wp->hold_time*1000))
			if (now - wpm->timestamp_firstinside_orbit >= cur_wp->param2 * 1000) {
				printf("Reached waypoint %u long enough \n", cur_wp->seq);

				if (cur_wp->autocontinue) {
					cur_wp->current = 0;

					if (wpm->current_active_wp_id == wpm->size - 1 && wpm->size > 1) {
						//the last waypoint was reached, if auto continue is
						//activated restart the waypoint list from the beginning
						wpm->current_active_wp_id = 1;

					} else {
						if ((uint16_t)(wpm->current_active_wp_id + 1) < wpm->size)
							wpm->current_active_wp_id++;
					}

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


int mavlink_waypoint_eventloop(uint64_t now, const struct vehicle_global_position_s *global_position, struct vehicle_local_position_s *local_position)
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

	// Do NOT continously send the current WP, since we're not loosing messages
	// if (now - wpm->timestamp_last_send_setpoint > wpm->delay_setpoint && wpm->current_active_wp_id < wpm->size) {
	// 	mavlink_wpm_send_setpoint(wpm->current_active_wp_id);
	// }

	check_waypoints_reached(now, global_position , local_position);

	return OK;
}


void mavlink_wpm_message_handler(const mavlink_message_t *msg, const struct vehicle_global_position_s *global_pos , struct vehicle_local_position_s *local_pos)
{
	uint64_t now = mavlink_missionlib_get_system_timestamp();

	switch (msg->msgid) {
//		case MAVLINK_MSG_ID_ATTITUDE:
//        {
//            if(msg->sysid == mavlink_system.sysid && wpm->current_active_wp_id < wpm->size)
//            {
//                mavlink_mission_item_t *wp = &(wpm->waypoints[wpm->current_active_wp_id]);
//                if(wp->frame == MAV_FRAME_LOCAL_ENU || wp->frame == MAV_FRAME_LOCAL_NED)
//                {
//                    mavlink_attitude_t att;
//                    mavlink_msg_attitude_decode(msg, &att);
//                    float yaw_tolerance = wpm->accept_range_yaw;
//                    //compare current yaw
//                    if (att.yaw - yaw_tolerance >= 0.0f && att.yaw + yaw_tolerance < 2.f*FM_PI)
//                    {
//                        if (att.yaw - yaw_tolerance <= wp->param4 && att.yaw + yaw_tolerance >= wp->param4)
//                            wpm->yaw_reached = true;
//                    }
//                    else if(att.yaw - yaw_tolerance < 0.0f)
//                    {
//                        float lowerBound = 360.0f + att.yaw - yaw_tolerance;
//                        if (lowerBound < wp->param4 || wp->param4 < att.yaw + yaw_tolerance)
//                            wpm->yaw_reached = true;
//                    }
//                    else
//                    {
//                        float upperBound = att.yaw + yaw_tolerance - 2.f*FM_PI;
//                        if (att.yaw - yaw_tolerance < wp->param4 || wp->param4 < upperBound)
//                            wpm->yaw_reached = true;
//                    }
//                }
//            }
//            break;
//        }
//
//		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
//        {
//            if(msg->sysid == mavlink_system.sysid && wpm->current_active_wp_id < wpm->size)
//            {
//                mavlink_mission_item_t *wp = &(wpm->waypoints[wpm->current_active_wp_id]);
//
//                if(wp->frame == MAV_FRAME_LOCAL_ENU || MAV_FRAME_LOCAL_NED)
//                {
//                    mavlink_local_position_ned_t pos;
//                    mavlink_msg_local_position_ned_decode(msg, &pos);
//                    //// if (debug) // printf("Received new position: x: %f | y: %f | z: %f\n", pos.x, pos.y, pos.z);
//
//                    wpm->pos_reached = false;
//
//                    // compare current position (given in message) with current waypoint
//                    float orbit = wp->param1;
//
//                    float dist;
////                    if (wp->param2 == 0)
////                    {
////						// FIXME segment distance
////                        //dist = mavlink_wpm_distance_to_segment(current_active_wp_id, pos.x, pos.y, pos.z);
////                    }
////                    else
////                    {
//                        dist = mavlink_wpm_distance_to_point(wpm->current_active_wp_id, pos.x, pos.y, pos.z);
////                    }
//
//                    if (dist >= 0.f && dist <= orbit && wpm->yaw_reached)
//                    {
//                        wpm->pos_reached = true;
//                    }
//                }
//            }
//            break;
//        }

//		case MAVLINK_MSG_ID_CMD: // special action from ground station
//        {
//            mavlink_cmd_t action;
//            mavlink_msg_cmd_decode(msg, &action);
//            if(action.target == mavlink_system.sysid)
//            {
//                // if (verbose) std::cerr << "Waypoint: received message with action " << action.action << std::endl;
//                switch (action.action)
//                {
//						//				case MAV_ACTION_LAUNCH:
//						//					// if (verbose) std::cerr << "Launch received" << std::endl;
//						//					current_active_wp_id = 0;
//						//					if (wpm->size>0)
//						//					{
//						//						setActive(waypoints[current_active_wp_id]);
//						//					}
//						//					else
//						//						// if (verbose) std::cerr << "No launch, waypointList empty" << std::endl;
//						//					break;
//
//						//				case MAV_ACTION_CONTINUE:
//						//					// if (verbose) std::c
//						//					err << "Continue received" << std::endl;
//						//					idle = false;
//						//					setActive(waypoints[current_active_wp_id]);
//						//					break;
//
//						//				case MAV_ACTION_HALT:
//						//					// if (verbose) std::cerr << "Halt received" << std::endl;
//						//					idle = true;
//						//					break;
//
//						//				default:
//						//					// if (verbose) std::cerr << "Unknown action received with id " << action.action << ", no action taken" << std::endl;
//						//					break;
//                }
//            }
//            break;
//        }

	case MAVLINK_MSG_ID_MISSION_ACK: {
			mavlink_mission_ack_t wpa;
			mavlink_msg_mission_ack_decode(msg, &wpa);

			if ((msg->sysid == wpm->current_partner_sysid && msg->compid == wpm->current_partner_compid) && (wpa.target_system == mavlink_system.sysid /*&& wpa.target_component == mavlink_wpm_comp_id*/)) {
				wpm->timestamp_lastaction = now;

				if (wpm->current_state == MAVLINK_WPM_STATE_SENDLIST || wpm->current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS) {
					if (wpm->current_wp_id == wpm->size - 1) {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("Got last WP ACK state -> IDLE");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Received ACK after having sent last waypoint, going to state MAVLINK_WPM_STATE_IDLE\n");

#endif
						wpm->current_state = MAVLINK_WPM_STATE_IDLE;
						wpm->current_wp_id = 0;
					}
				}

			} else {
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: curr partner id mismatch");
#else

				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");

#endif
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

#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("NEW WP SET");
#else

						if (MAVLINK_WPM_VERBOSE) printf("New current waypoint %u\n", wpm->current_active_wp_id);

#endif
						wpm->yaw_reached = false;
						wpm->pos_reached = false;
						mavlink_wpm_send_waypoint_current(wpm->current_active_wp_id);
						mavlink_wpm_send_setpoint(wpm->current_active_wp_id);
						wpm->timestamp_firstinside_orbit = 0;

					} else {
#ifdef MAVLINK_WPM_NO_PRINTF
						mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Not in list");
#else

						if (MAVLINK_WPM_VERBOSE) printf("Ignored MAVLINK_MSG_ID_MISSION_ITEM_SET_CURRENT: Index out of bounds\n");

#endif
					}

				} else {
#ifdef MAVLINK_WPM_NO_PRINTF
					mavlink_missionlib_send_gcs_string("IGN WP CURR CMD: Busy");
#else

					if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE NOT IN IDLE STATE\n");

#endif
				}

			} else {
#ifdef MAVLINK_WPM_NO_PRINTF
				mavlink_missionlib_send_gcs_string("REJ. WP CMD: target id mismatch");
#else

				if (MAVLINK_WPM_VERBOSE) printf("IGNORED WAYPOINT COMMAND BECAUSE TARGET SYSTEM AND COMPONENT OR COMM PARTNER ID MISMATCH\n");

#endif
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

	check_waypoints_reached(now, global_pos, local_pos);
}
