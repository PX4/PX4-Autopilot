/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.cpp
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "mission_block.h"
#include "navigator.h"

#include <math.h>
#include <float.h>

#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vtol_vehicle_status.h>

using matrix::wrap_pi;

MissionBlock::MissionBlock(Navigator *navigator, uint8_t navigator_state_id) :
	NavigatorMode(navigator, navigator_state_id)
{

}

bool
MissionBlock::is_mission_item_reached_or_completed()
{
	const hrt_abstime now = hrt_absolute_time();

	// Handle indefinite waypoints and action commands
	switch (_mission_item.nav_cmd) {

	// Action Commands that doesn't have timeout completes instantaneously
	case NAV_CMD_DO_SET_ACTUATOR:
	case NAV_CMD_DO_LAND_START:
	case NAV_CMD_DO_TRIGGER_CONTROL:
	case NAV_CMD_DO_DIGICAM_CONTROL:
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
	case NAV_CMD_DO_CONTROL_VIDEO:
	case NAV_CMD_DO_MOUNT_CONFIGURE:
	case NAV_CMD_DO_MOUNT_CONTROL:
	case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
	case NAV_CMD_DO_SET_ROI:
	case NAV_CMD_DO_SET_ROI_LOCATION:
	case NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case NAV_CMD_DO_SET_ROI_NONE:
	case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
	case NAV_CMD_OBLIQUE_SURVEY:
	case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case NAV_CMD_SET_CAMERA_MODE:
	case NAV_CMD_SET_CAMERA_SOURCE:
	case NAV_CMD_SET_CAMERA_ZOOM:
	case NAV_CMD_SET_CAMERA_FOCUS:
	case NAV_CMD_DO_CHANGE_SPEED:
	case NAV_CMD_DO_SET_HOME:
		return true;

	// Indefinite Waypoints
	case NAV_CMD_LAND: /* fall through */
	case NAV_CMD_VTOL_LAND:
		return _navigator->get_land_detected()->landed;

	case NAV_CMD_IDLE: /* fall through */
	case NAV_CMD_LOITER_UNLIMITED:
		return false;

	case NAV_CMD_DO_VTOL_TRANSITION:

		if (int(_mission_item.params[0]) == 3) {
			// transition to RW requested, only accept waypoint if vehicle state has changed accordingly
			return _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			       && !_navigator->get_vstatus()->in_transition_mode;

		} else if (int(_mission_item.params[0]) == 4) {
			// transition to FW requested, only accept waypoint if vehicle state has changed accordingly
			return _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

		} else {
			// invalid vtol transition request
			return false;
		}

	case NAV_CMD_VTOL_TAKEOFF:
		if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			return true;
		}

		break;

	case NAV_CMD_DELAY:
		// Set reached flags directly such that only the delay time is considered
		_waypoint_position_reached = true;
		_waypoint_yaw_reached = true;

		// Set timestamp when entering only (it's reset to 0 for every waypoint)
		if (_time_wp_reached == 0) {
			_time_wp_reached = now;
		}

		break;

	case NAV_CMD_DO_WINCH:
	case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
	case NAV_CMD_DO_GRIPPER:
		if (now > _timestamp_command_timeout + (_command_timeout * 1_s)) {
			return true;
		}

		return false; // Still waiting

	default:
		/* do nothing, this is a 3D waypoint */
		break;
	}

	// Update the 'waypoint position reached' status
	if (!_navigator->get_land_detected()->landed && !_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		const float mission_item_altitude_amsl = get_absolute_altitude_for_item(_mission_item);

		// consider mission_item.loiter_radius invalid if NAN or 0, use default value in this case.
		const float mission_item_loiter_radius_abs = (PX4_ISFINITE(_mission_item.loiter_radius)
				&& fabsf(_mission_item.loiter_radius) > FLT_EPSILON) ? fabsf(_mission_item.loiter_radius) :
				_navigator->get_loiter_radius();

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, mission_item_altitude_amsl,
				_navigator->get_global_position()->lat,
				_navigator->get_global_position()->lon,
				_navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		if ((_mission_item.nav_cmd == NAV_CMD_TAKEOFF || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)
		    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

			/* We want to avoid the edge case where the acceptance radius is bigger or equal than
			 * the altitude of the takeoff waypoint above home. Otherwise, we do not really follow
			 * take-off procedures like leaving the landing gear down. */

			float takeoff_alt = _mission_item.altitude_is_relative ?
					    _mission_item.altitude :
					    (_mission_item.altitude - _navigator->get_home_position()->alt);

			float altitude_acceptance_radius = _navigator->get_altitude_acceptance_radius();

			/* It should be safe to just use half of the takoeff_alt as an acceptance radius. */
			if (takeoff_alt > 0 && takeoff_alt < altitude_acceptance_radius) {
				altitude_acceptance_radius = takeoff_alt / 2.0f;
			}

			/* require only altitude for takeoff for multicopter */
			if (_navigator->get_global_position()->alt >
			    mission_item_altitude_amsl - altitude_acceptance_radius) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			   && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			/* fixed-wing takeoff is reached once the vehicle has exceeded the takeoff altitude */
			if (_navigator->get_global_position()->alt > mission_item_altitude_amsl) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			   && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER) {
			// Accept takeoff waypoint to be reached if the distance in 2D plane is within acceptance radius
			if (dist_xy >= 0.0f && dist_xy <= _navigator->get_acceptance_radius()) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) {
			// For takeoff mission items use the parameter for the takeoff acceptance radius
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius()
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				_waypoint_position_reached = true;
			}

		} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING &&
			   (_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT)) {

			/* Loiter mission item on a non rotary wing: the aircraft is going to circle the
			 * coordinates with a radius equal to the loiter_radius field. It is not flying
			 * through the waypoint center.
			 * Therefore the item is marked as reached once the system reaches the loiter
			 * radius + navigation switch distance. Time inside and turn count is handled elsewhere.
			 */

			// check if within loiter radius around wp, if yes then set altitude sp to mission item
			if (dist >= 0.0f && dist_xy <= (_navigator->get_acceptance_radius() + mission_item_loiter_radius_abs)
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
			// NAV_CMD_LOITER_TO_ALT only uses mission item altitude once it's in the loiter.
			// First check if the altitude setpoint is the mission setpoint (that means that the loiter is not yet reached)
			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			if (fabsf(curr_sp->alt - mission_item_altitude_amsl) >= FLT_EPSILON) {
				dist_xy = -1.0f;
				dist_z = -1.0f;

				dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, curr_sp->alt,
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				// check if within loiter radius around wp, if yes then set altitude sp to mission item
				if (dist >= 0.0f && dist_xy <= (_navigator->get_acceptance_radius() + mission_item_loiter_radius_abs)
				    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

					curr_sp->alt = mission_item_altitude_amsl;
					curr_sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					_navigator->set_position_setpoint_triplet_updated();
				}

			} else if (dist >= 0.f && dist_xy <= (_navigator->get_acceptance_radius() + mission_item_loiter_radius_abs)
				   && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				// loitering, check if new altitude is reached, while still also having check on position

				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_CONDITION_GATE) {

			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			// if the setpoint is valid we are checking if we reached the gate
			// in the case of an invalid setpoint we are defaulting to
			// assuming that we have already reached the gate to not block
			// the further execution of the mission.
			if (curr_sp->valid) {

				// location of gate (mission item)
				MapProjection ref_pos{_mission_item.lat, _mission_item.lon};

				// current setpoint
				matrix::Vector2f gate_to_curr_sp = ref_pos.project(curr_sp->lat, curr_sp->lon);

				// system position
				matrix::Vector2f vehicle_pos = ref_pos.project(_navigator->get_global_position()->lat,
							       _navigator->get_global_position()->lon);
				const float dot_product = vehicle_pos.dot(gate_to_curr_sp.normalized());

				// if the dot product (projected vector) is positive, then
				// the current position is between the gate position and the
				// next waypoint
				if (dot_product >= 0) {
					_waypoint_position_reached = true;
					_waypoint_yaw_reached = true;
					_time_wp_reached = now;
				}
			}

		} else {

			float acceptance_radius = _navigator->get_acceptance_radius();

			// We use the acceptance radius of the mission item if it has been set (not NAN)
			// but only for multicopter.
			if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
			    && PX4_ISFINITE(_mission_item.acceptance_radius) && _mission_item.acceptance_radius > FLT_EPSILON) {
				acceptance_radius = _mission_item.acceptance_radius;
			}

			float alt_acc_rad_m = _navigator->get_altitude_acceptance_radius();

			/* for vtol back transition calculate acceptance radius based on time and ground speed */
			if (_mission_item.vtol_back_transition
			    && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

				float velocity = sqrtf(_navigator->get_local_position()->vx * _navigator->get_local_position()->vx +
						       _navigator->get_local_position()->vy * _navigator->get_local_position()->vy);

				const float back_trans_dec = _navigator->get_vtol_back_trans_deceleration();

				if (back_trans_dec > FLT_EPSILON && velocity > FLT_EPSILON) {
					acceptance_radius = (velocity / back_trans_dec / 2) * velocity;

				}

				// do not care for altitude when approaching the backtransition point. Not accepting the waypoint causes
				// the vehicle to perform a sharp turn after passing the land waypoint and this causes worse unexected behavior
				alt_acc_rad_m = INFINITY;

			}

			bool passed_curr_wp = false;

			if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {

				const float dist_prev_to_curr = get_distance_to_next_waypoint(_navigator->get_position_setpoint_triplet()->previous.lat,
								_navigator->get_position_setpoint_triplet()->previous.lon, _navigator->get_position_setpoint_triplet()->current.lat,
								_navigator->get_position_setpoint_triplet()->current.lon);

				if (dist_prev_to_curr > 1.0e-6f && _navigator->get_position_setpoint_triplet()->previous.valid) {
					// Fixed-wing guidance interprets this condition as line segment following

					// vector from previous waypoint to current waypoint
					float vector_prev_to_curr_north;
					float vector_prev_to_curr_east;
					get_vector_to_next_waypoint_fast(_navigator->get_position_setpoint_triplet()->previous.lat,
									 _navigator->get_position_setpoint_triplet()->previous.lon, _navigator->get_position_setpoint_triplet()->current.lat,
									 _navigator->get_position_setpoint_triplet()->current.lon, &vector_prev_to_curr_north,
									 &vector_prev_to_curr_east);

					// vector from next waypoint to aircraft
					float vector_curr_to_vehicle_north;
					float vector_curr_to_vehicle_east;
					get_vector_to_next_waypoint_fast(_navigator->get_position_setpoint_triplet()->current.lat,
									 _navigator->get_position_setpoint_triplet()->current.lon, _navigator->get_global_position()->lat,
									 _navigator->get_global_position()->lon, &vector_curr_to_vehicle_north,
									 &vector_curr_to_vehicle_east);

					// if dot product of vectors is positive, we are passed the current waypoint (the terminal point on the line segment) and should switch to next mission item
					passed_curr_wp = vector_prev_to_curr_north * vector_curr_to_vehicle_north + vector_prev_to_curr_east *
							 vector_curr_to_vehicle_east > 0.0f;
				}
			}

			if (dist_xy >= 0.0f && (dist_xy <= acceptance_radius || passed_curr_wp) && dist_z <= alt_acc_rad_m) {
				_waypoint_position_reached = true;
			}
		}

		if (_waypoint_position_reached) {
			// reached just now
			_time_wp_reached = now;
		}

		// consider yaw reached for non-rotary wing vehicles (such as fixed-wing)
		if (_navigator->get_vstatus()->vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			_waypoint_yaw_reached = true;
		}
	}

	// Update the 'waypoint position reached' status (only for rotary wing flight)
	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && _navigator->get_yaw_to_be_accepted(_mission_item.yaw)
		    && _navigator->get_local_position()->heading_good_for_control) {

			const float yaw_err = wrap_pi(_mission_item.yaw - _navigator->get_local_position()->heading);

			/* accept yaw if reached or if timeout is set in which case we ignore not forced headings */
			if (fabsf(yaw_err) < _navigator->get_yaw_threshold()
			    || (_navigator->get_yaw_timeout() >= FLT_EPSILON && !_mission_item.force_heading)) {

				_waypoint_yaw_reached = true;
			}

			/* if heading needs to be reached, the timeout is enabled and we don't make it, abort mission */
			if (!_waypoint_yaw_reached && _mission_item.force_heading &&
			    (_navigator->get_yaw_timeout() >= FLT_EPSILON) &&
			    (now - _time_wp_reached >= (hrt_abstime)_navigator->get_yaw_timeout() * 1e6f)) {

				_navigator->set_mission_failure_heading_timeout();
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	// Handle Loiter/Delay Timeout if the waypoint position and yaw setpoint got reached
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		bool time_inside_reached = false;

		/* check if the MAV was long enough inside the waypoint orbit */
		if ((get_time_inside(_mission_item) < FLT_EPSILON) ||
		    (now >= (hrt_abstime)(get_time_inside(_mission_item) * 1_s) + _time_wp_reached)) {
			time_inside_reached = true;
		}

		// check if course for exit is reached (only applies for fixed-wing flight)
		bool exit_course_reached = false;

		if (time_inside_reached) {

			struct position_setpoint_s *curr_sp_new = &_navigator->get_position_setpoint_triplet()->current;
			const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

			const float dist_current_next = get_distance_to_next_waypoint(curr_sp_new->lat, curr_sp_new->lon, next_sp.lat,
							next_sp.lon);

			/* enforce exit course if in FW, the next wp is valid, the vehicle is currently loitering and either having force_heading set,
			   or if loitering to achieve altitdue at a NAV_CMD_WAYPOINT */
			const bool enforce_exit_course = _navigator->get_vstatus()->vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
							 && next_sp.valid
							 && curr_sp_new->type == position_setpoint_s::SETPOINT_TYPE_LOITER
							 && (_mission_item.force_heading || _mission_item.nav_cmd == NAV_CMD_WAYPOINT);

			// can only enforce exit course if next waypoint is not within loiter radius of current waypoint (with small margin)
			const bool exit_course_is_reachable = dist_current_next > 1.05f * curr_sp_new->loiter_radius;

			if (enforce_exit_course && exit_course_is_reachable) {

				float vehicle_position_to_next_waypoint_north;
				float vehicle_position_to_next_waypoint_east;
				get_vector_to_next_waypoint(_navigator->get_global_position()->lat, _navigator->get_global_position()->lon, next_sp.lat,
							    next_sp.lon, &vehicle_position_to_next_waypoint_north,  &vehicle_position_to_next_waypoint_east);

				// this vector defines the exit bearing
				const matrix::Vector2f vector_vehicle_position_to_next_waypoint = {vehicle_position_to_next_waypoint_north, vehicle_position_to_next_waypoint_east};

				const matrix::Vector2f vehicle_ground_velocity = {_navigator->get_local_position()->vx, _navigator->get_local_position()->vy};

				exit_course_reached = vector_vehicle_position_to_next_waypoint.dot(vehicle_ground_velocity) >
						      vector_vehicle_position_to_next_waypoint.norm() * vehicle_ground_velocity.norm() * kCosineExitCourseThreshold;

			} else {
				exit_course_reached = true;
			}
		}

		// set exit flight course to next waypoint
		if (exit_course_reached) {
			position_setpoint_s &curr_sp = _navigator->get_position_setpoint_triplet()->current;
			const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

			const float range = get_distance_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

			// exit xtrack location
			// reset lat/lon of loiter waypoint so vehicle follows a tangent
			if (_mission_item.loiter_exit_xtrack && next_sp.valid && PX4_ISFINITE(range) &&
			    (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			     _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT)) {

				float bearing = get_bearing_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

				// calculate (positive) angle between current bearing vector (orbit center to next waypoint) and vector pointing to tangent exit location
				const float ratio = math::min(fabsf(curr_sp.loiter_radius / range), 1.0f);
				float inner_angle = acosf(ratio);

				// Compute "ideal" tangent origin
				if (curr_sp.loiter_direction_counter_clockwise) {
					bearing += inner_angle;

				} else {
					bearing -= inner_angle;
				}

				// set typ to position, will get set to loiter in the fw position controller once close
				// and replace current setpoint lat/lon with tangent coordinate
				curr_sp.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
				waypoint_from_heading_and_distance(curr_sp.lat, curr_sp.lon,
								   bearing, fabsf(curr_sp.loiter_radius),
								   &curr_sp.lat, &curr_sp.lon);
			}

			return true; // mission item is reached
		}
	}

	return false;
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_wp_reached = 0;
}

void
MissionBlock::issue_command(const mission_item_s &item)
{
	if (item_contains_position(item)
	    || item_contains_gate(item)
	    || item_contains_marker(item)) {
		return;
	}

	// This is to support legacy DO_MOUNT_CONTROL as part of a mission.
	if (item.nav_cmd == NAV_CMD_DO_MOUNT_CONTROL) {
		_navigator->acquire_gimbal_control();
	}

	// Mission item's NAV_CMD enums directly map to the according vehicle command
	// So set the raw value directly (MAV_FRAME_MISSION mission item)
	vehicle_command_s vehicle_command{};
	vehicle_command.command = item.nav_cmd;
	vehicle_command.param1 = item.params[0];
	vehicle_command.param2 = item.params[1];
	vehicle_command.param3 = item.params[2];
	vehicle_command.param4 = item.params[3];
	vehicle_command.param5 = static_cast<double>(item.params[4]);
	vehicle_command.param6 = static_cast<double>(item.params[5]);
	vehicle_command.param7 = item.params[6];

	if (item.nav_cmd == NAV_CMD_DO_SET_ROI_LOCATION) {
		// We need to send out the ROI location that was parsed potentially with double precision to lat/lon because mission item parameters 5 and 6 only have float precision
		vehicle_command.param5 = item.lat;
		vehicle_command.param6 = item.lon;

		if (item.altitude_is_relative) {
			vehicle_command.param7 = item.altitude + _navigator->get_home_position()->alt;
		}
	}

	_navigator->publish_vehicle_command(vehicle_command);

	if (item_has_timeout(item)) {
		_timestamp_command_timeout = hrt_absolute_time();
	}
}

float
MissionBlock::get_time_inside(const mission_item_s &item) const
{
	if ((item.nav_cmd == NAV_CMD_WAYPOINT
	     && _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) ||
	    item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	    item.nav_cmd == NAV_CMD_DELAY) {

		// a negative time inside would be invalid
		return math::max(item.time_inside, 0.0f);
	}

	return 0.0f;
}

// TODO: get_time_inside and item_has_timeout is quite redundant. Separate them out
// Problem arises from the fact that DO_WINCH and DO_GRIPPER *should be an instantaneous command,
// and shouldn't have a timeout defined as it is a DO_* command. It should rather be defined as CONDITION_GRIPPER
// or so, and have a function named 'item_is_conditional'
// Reference: https://mavlink.io/en/services/mission.html#mavlink_commands
// A similar condition applies to DO_GIMBAL_MANAGER_PITCHYAW
bool
MissionBlock::item_has_timeout(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_DO_WINCH ||
	       item.nav_cmd == NAV_CMD_DO_GRIPPER ||
	       item.nav_cmd == NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
}

bool
MissionBlock::item_contains_position(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_WAYPOINT ||
	       item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
	       item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	       item.nav_cmd == NAV_CMD_LAND ||
	       item.nav_cmd == NAV_CMD_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_LOITER_TO_ALT ||
	       item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_VTOL_LAND;
}

bool
MissionBlock::item_contains_gate(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_CONDITION_GATE;
}

bool
MissionBlock::item_contains_marker(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_DO_LAND_START;
}

bool
MissionBlock::mission_item_to_position_setpoint(const mission_item_s &item, position_setpoint_s *sp)
{
	// Don't change the setpoint for non-position items
	if (!item_contains_position(item)) {
		return false;
	}

	sp->lat = item.lat;
	sp->lon = item.lon;
	sp->alt = get_absolute_altitude_for_item(item);
	sp->yaw = item.yaw;
	sp->loiter_radius = (fabsf(item.loiter_radius) > FLT_EPSILON) ? fabsf(item.loiter_radius) :
			    _navigator->get_loiter_radius();
	sp->loiter_direction_counter_clockwise = item.loiter_radius < 0;

	if (item.acceptance_radius > FLT_EPSILON && PX4_ISFINITE(item.acceptance_radius)) {
		// if the mission item has a specified acceptance radius, overwrite the default one from parameters
		sp->acceptance_radius = item.acceptance_radius;

	} else {
		sp->acceptance_radius = _navigator->get_default_acceptance_radius();
	}

	// by default, FW guidance logic will take alt acceptance from NAV_FW_ALT_RAD, in some special cases
	// we override it after this
	sp->alt_acceptance_radius = NAN;

	sp->cruising_speed = _navigator->get_cruising_speed();
	sp->cruising_throttle = _navigator->get_cruising_throttle();

	// for fixed wing we don't use cruising_throttle directly anymore, instead we command airspeed setpoints via cruising_speed
	// we still use cruising throttle here to determine if gliding is enabled
	sp->gliding_enabled = (_navigator->get_cruising_throttle() < FLT_EPSILON);

	switch (item.nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:
	case NAV_CMD_VTOL_TAKEOFF:

		// if already flying (armed and !landed) treat TAKEOFF like regular POSITION
		if ((_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !_navigator->get_land_detected()->landed && !_navigator->get_land_detected()->maybe_landed) {

			sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		} else {
			sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

			// Don't set a yaw setpoint for takeoff, as Navigator doesn't handle the yaw reset.
			// The yaw setpoint generation is handled by FlightTaskAuto.
			sp->yaw = NAN;
		}

		break;

	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LAND;
		break;

	case NAV_CMD_LOITER_TO_ALT:
		sp->alt = _navigator->get_global_position()->alt;

	// FALLTHROUGH
	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_UNLIMITED:

		sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		break;

	default:
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;
	}

	sp->valid = true;
	sp->timestamp = hrt_absolute_time();

	return sp->valid;
}

void
MissionBlock::setLoiterItemFromCurrentPositionSetpoint(struct mission_item_s *item)
{
	setLoiterItemCommonFields(item);

	const position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	item->lat = pos_sp_triplet->current.lat;
	item->lon = pos_sp_triplet->current.lon;
	item->altitude = pos_sp_triplet->current.alt;
	item->loiter_radius = pos_sp_triplet->current.loiter_direction_counter_clockwise ?
			      -pos_sp_triplet->current.loiter_radius : pos_sp_triplet->current.loiter_radius;
	item->yaw = pos_sp_triplet->current.yaw;
}

void
MissionBlock::setLoiterItemFromCurrentPosition(struct mission_item_s *item)
{
	setLoiterItemCommonFields(item);

	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;

	// check if minimum loiter altitude is specified, and enforce it if so
	float loiter_altitude_amsl = _navigator->get_global_position()->alt;

	if (_navigator->get_loiter_min_alt() > FLT_EPSILON) {
		loiter_altitude_amsl = math::max(loiter_altitude_amsl,
						 _navigator->get_home_position()->alt + _navigator->get_loiter_min_alt());
	}

	item->altitude = loiter_altitude_amsl;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->yaw = NAN;
}

void
MissionBlock::setLoiterItemFromCurrentPositionWithBraking(struct mission_item_s *item)
{
	setLoiterItemCommonFields(item);

	_navigator->preproject_stop_point(item->lat, item->lon);

	item->altitude = _navigator->get_global_position()->alt;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->yaw = NAN;
}

void
MissionBlock::setLoiterItemCommonFields(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

	item->altitude_is_relative = false;
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->yaw = NAN;
	item->time_inside = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_takeoff_item(struct mission_item_s *item, float abs_altitude)
{
	item->nav_cmd = NAV_CMD_TAKEOFF;

	/* use current position */
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->yaw = NAN;

	item->altitude = abs_altitude;
	item->altitude_is_relative = false;

	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->loiter_radius = _navigator->get_loiter_radius();
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_land_item(struct mission_item_s *item)
{
	/* VTOL transition to RW before landing */
	if (_navigator->force_vtol()) {

		vehicle_command_s vehicle_command{};
		vehicle_command.command = NAV_CMD_DO_VTOL_TRANSITION;
		vehicle_command.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		vehicle_command.param2 = 0.f; // normal unforced transition
		_navigator->publish_vehicle_command(vehicle_command);
	}

	/* set the land item */
	item->nav_cmd = NAV_CMD_LAND;

	// set land item to current position
	if (_navigator->get_local_position()->xy_global) {
		item->lat = _navigator->get_global_position()->lat;
		item->lon = _navigator->get_global_position()->lon;

	} else {
		item->lat = (double)NAN;
		item->lon = (double)NAN;
	}

	item->yaw = NAN;

	item->altitude = 0;
	item->altitude_is_relative = false;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_idle_item(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_IDLE;
	item->lat = _navigator->get_home_position()->lat;
	item->lon = _navigator->get_home_position()->lon;
	item->altitude_is_relative = false;
	item->altitude = _navigator->get_home_position()->alt;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_vtol_transition_item(struct mission_item_s *item, const uint8_t new_mode)
{
	item->nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item->params[0] = (float) new_mode;
	item->params[1] = 0.0f; // not immediate transition
	item->autocontinue = true;
}

float
MissionBlock::get_absolute_altitude_for_item(const mission_item_s &mission_item) const
{
	return get_absolute_altitude_for_item(mission_item, _navigator->get_home_position()->alt);
}

float
MissionBlock::get_absolute_altitude_for_item(const mission_item_s &mission_item, float home_alt)
{
	if (mission_item.altitude_is_relative) {
		return mission_item.altitude + home_alt;

	} else {
		return mission_item.altitude;
	}
}

void
MissionBlock::copy_position_if_valid(struct mission_item_s *const mission_item,
				     const struct position_setpoint_s *const setpoint) const
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		mission_item->lat = setpoint->lat;
		mission_item->lon = setpoint->lon;
		mission_item->altitude = setpoint->alt;

	} else {
		mission_item->lat = _navigator->get_global_position()->lat;
		mission_item->lon = _navigator->get_global_position()->lon;
		mission_item->altitude = _navigator->get_global_position()->alt;
	}

	mission_item->altitude_is_relative = false;
}

void
MissionBlock::set_align_mission_item(struct mission_item_s *const mission_item,
				     const struct mission_item_s *const mission_item_next) const
{
	mission_item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_position_if_valid(mission_item, &(_navigator->get_position_setpoint_triplet()->current));
	mission_item->altitude_is_relative = false;
	mission_item->autocontinue = true;
	mission_item->time_inside = 0.0f;
	mission_item->yaw = get_bearing_to_next_waypoint(
				    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
				    mission_item_next->lat, mission_item_next->lon);
	mission_item->force_heading = true;
}

void
MissionBlock::initialize()
{
	_mission_item.lat = (double)NAN;
	_mission_item.lon = (double)NAN;
	_mission_item.yaw = NAN;
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_ONBOARD;
}

void MissionBlock::setLoiterToAltMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp,
		float loiter_radius) const
{
	item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
	item.lat = pos_yaw_sp.lat;
	item.lon = pos_yaw_sp.lon;
	item.altitude = pos_yaw_sp.alt;
	item.altitude_is_relative = false;
	item.yaw = pos_yaw_sp.yaw;

	item.acceptance_radius = _navigator->get_acceptance_radius();
	item.time_inside = 0.0f;
	item.autocontinue = true;
	item.origin = ORIGIN_ONBOARD;
	item.loiter_radius = loiter_radius;
}

void MissionBlock::setLoiterHoldMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp,
		float loiter_time, float loiter_radius) const
{
	const bool autocontinue = (loiter_time > -FLT_EPSILON);

	if (autocontinue) {
		item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;

	} else {
		item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	}

	item.lat = pos_yaw_sp.lat;
	item.lon = pos_yaw_sp.lon;
	item.altitude = pos_yaw_sp.alt;
	item.altitude_is_relative = false;

	item.yaw = NAN;

	item.acceptance_radius = _navigator->get_acceptance_radius();
	item.time_inside = math::max(loiter_time, 0.0f);
	item.autocontinue = autocontinue;
	item.origin = ORIGIN_ONBOARD;
	item.loiter_radius = loiter_radius;
}

void MissionBlock::setMoveToPositionMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp) const
{
	item.nav_cmd = NAV_CMD_WAYPOINT;
	item.lat = pos_yaw_sp.lat;
	item.lon = pos_yaw_sp.lon;
	item.altitude = pos_yaw_sp.alt;
	item.altitude_is_relative = false;

	item.autocontinue = true;
	item.acceptance_radius = _navigator->get_acceptance_radius();
	item.time_inside = 0.f;
	item.origin = ORIGIN_ONBOARD;

	item.yaw = pos_yaw_sp.yaw;
}

void MissionBlock::setLandMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp) const
{
	item.nav_cmd = NAV_CMD_LAND;
	item.lat = pos_yaw_sp.lat;
	item.lon = pos_yaw_sp.lon;
	item.altitude = pos_yaw_sp.alt;
	item.yaw = pos_yaw_sp.yaw;
	item.acceptance_radius = _navigator->get_acceptance_radius();
	item.time_inside = 0.0f;
	item.autocontinue = true;
	item.origin = ORIGIN_ONBOARD;
}

void MissionBlock::startPrecLand(uint16_t land_precision)
{
	if (_mission_item.land_precision == 1) {
		_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);
		_navigator->get_precland()->on_activation();

	} else if (_mission_item.land_precision == 2) {
		_navigator->get_precland()->set_mode(PrecLandMode::Required);
		_navigator->get_precland()->on_activation();
	}
}

void MissionBlock::updateAltToAvoidTerrainCollisionAndRepublishTriplet(mission_item_s mission_item)
{
	// Avoid flying into terrain using the distance sensor. Enable through the parameter NAV_MIN_GND_DIST.
	// Only active during commanded descents with vz>0 (to prevent climb-aways), excluding landing and VTOL transitions.
	// It changes the altitude setpoint in the triplet to maintain the current altitude and republish the triplet.
	// We also change the mission item altitude used for acceptance calculations to prevent getting stuck in a loop.

	// This threshold is needed to prevent the check from re-triggering due to small altitude over-shoots while
	// tracking the new altitude setpoint.
	static constexpr float kAltitudeDifferenceForDescentCondition = 2.f;


	if (_navigator->get_nav_min_gnd_dist_param() > FLT_EPSILON && _mission_item.nav_cmd != NAV_CMD_LAND
	    && _mission_item.nav_cmd != NAV_CMD_VTOL_LAND && _mission_item.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION
	    && _mission_item.nav_cmd != NAV_CMD_IDLE
	    && _navigator->get_local_position()->dist_bottom_valid
	    && _navigator->get_local_position()->dist_bottom < _navigator->get_nav_min_gnd_dist_param()
	    && _navigator->get_local_position()->vz > FLT_EPSILON
	    && _navigator->get_global_position()->alt - get_absolute_altitude_for_item(mission_item) >
	    kAltitudeDifferenceForDescentCondition) {

		_navigator->sendWarningDescentStoppedDueToTerrain();

		struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;
		curr_sp->alt = _navigator->get_global_position()->alt;
		_navigator->set_position_setpoint_triplet_updated();

		_mission_item.altitude = _navigator->get_global_position()->alt;
		_mission_item.altitude_is_relative = false;
	}
}

void MissionBlock::updateFailsafeChecks()
{
	updateMaxHaglFailsafe();
}

void MissionBlock::updateMaxHaglFailsafe()
{
	const float target_alt = _navigator->get_position_setpoint_triplet()->current.alt;

	if (_navigator->get_global_position()->terrain_alt_valid
	    && ((target_alt - _navigator->get_global_position()->terrain_alt)
		> math::min(_navigator->get_local_position()->hagl_max_z, _navigator->get_local_position()->hagl_max_xy))) {
		// Handle case where the altitude setpoint is above the maximum HAGL (height above ground level)
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Target altitude higher than max HAGL\t");
		events::send(events::ID("navigator_fail_max_hagl"), events::Log::Error, "Target altitude higher than max HAGL");

		_navigator->trigger_hagl_failsafe(getNavigatorStateId());

		// While waiting for a failsafe action from commander, keep the curren position
		setLoiterItemFromCurrentPosition(&_mission_item);

		mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);

		_navigator->set_position_setpoint_triplet_updated();
	}
}
