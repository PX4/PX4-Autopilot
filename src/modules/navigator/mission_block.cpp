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

#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vtol_vehicle_status.h>

using matrix::wrap_pi;

MissionBlock::MissionBlock(Navigator *navigator) :
	NavigatorMode(navigator)
{
	_mission_item.lat = NAN;
	_mission_item.lon = NAN;

	_mission_item.yaw = NAN;

	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.time_inside = 0.0f;
	_mission_item.autocontinue = true;
	_mission_item.origin = ORIGIN_ONBOARD;
}

bool
MissionBlock::is_mission_item_reached()
{
	switch (_mission_item.nav_cmd) {
	case NAV_CMD_DO_SET_SERVO:
		return true;

	case NAV_CMD_LAND: /* fall through */
	case NAV_CMD_VTOL_LAND:
		return _navigator->get_land_detected()->landed;

	case NAV_CMD_IDLE: /* fall through */
	case NAV_CMD_LOITER_UNLIMITED:
		return false;

	case NAV_CMD_DO_LAND_START:
	case NAV_CMD_DO_TRIGGER_CONTROL:
	case NAV_CMD_DO_DIGICAM_CONTROL:
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
	case NAV_CMD_DO_MOUNT_CONFIGURE:
	case NAV_CMD_DO_MOUNT_CONTROL:
	case NAV_CMD_DO_SET_ROI:
	case NAV_CMD_DO_SET_ROI_LOCATION:
	case NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case NAV_CMD_DO_SET_ROI_NONE:
	case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
	case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case NAV_CMD_SET_CAMERA_MODE:
		return true;

	case NAV_CMD_DO_VTOL_TRANSITION:

		/*
		 * We wait half a second to give the transition command time to propagate.
		 * Then monitor the transition status for completion.
		 */
		// TODO: check desired transition state achieved and drop _action_start
		if (hrt_absolute_time() - _action_start > 500000 &&
		    !_navigator->get_vstatus()->in_transition_mode) {

			_action_start = 0;
			return true;

		} else {
			return false;
		}

	case NAV_CMD_DO_CHANGE_SPEED:
	case NAV_CMD_DO_SET_HOME:
		return true;

	default:
		if (!_navigator->get_land_detected()->landed) {
			if (position_achieved(_mission_item) && yaw_achieved(_mission_item) && time_inside_finished(_mission_item)) {
				set_loiter_exit(_mission_item);
				return true;
			}
		}
	}

	// all acceptance criteria must be met in the same iteration
	return false;
}

bool
MissionBlock::position_achieved(const mission_item_s &item)
{
	bool waypoint_position_reached = false;

	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	/* for normal mission items used their acceptance radius */
	float mission_acceptance_radius = _navigator->get_acceptance_radius(item.acceptance_radius);

	/* if set to zero use the default instead */
	if (mission_acceptance_radius < NAV_EPSILON_POSITION) {
		mission_acceptance_radius = _navigator->get_acceptance_radius();
	}

	float altitude_acceptance_radius = _navigator->get_altitude_acceptance_radius();
	const float altitude_amsl = get_absolute_altitude_for_item(item);

	float dist_xy = -1.0f;
	float dist_z = -1.0f;
	const float dist = get_distance_to_point_global_wgs84(item.lat, item.lon, altitude_amsl,
			   gpos.lat, gpos.lon, gpos.alt, &dist_xy, &dist_z);

	switch (item.nav_cmd) {
	case NAV_CMD_VTOL_TAKEOFF:
	case NAV_CMD_TAKEOFF: {
			if (_navigator->get_vstatus()->is_rotary_wing) {
				/* We want to avoid the edge case where the acceptance radius is bigger or equal than
				 * the altitude of the takeoff waypoint above home. Otherwise, we do not really follow
				 * take-off procedures like leaving the landing gear down. */

				float takeoff_alt = item.altitude_is_relative ? item.altitude : (item.altitude - _navigator->get_home_position()->alt);

				/* It should be safe to just use half of the takoeff_alt as an acceptance radius. */
				if (takeoff_alt > 0 && takeoff_alt < altitude_acceptance_radius) {
					altitude_acceptance_radius = takeoff_alt / 2.0f;
				}

				/* require only altitude for takeoff for multicopter */
				if (gpos.alt > (altitude_amsl - altitude_acceptance_radius)) {
					waypoint_position_reached = true;
				}

			} else {
				/* for takeoff mission items use the parameter for the takeoff acceptance radius */
				if (dist >= 0.0f &&
				    dist <= _navigator->get_acceptance_radius() &&
				    dist_z <= altitude_acceptance_radius) {

					waypoint_position_reached = true;
				}
			}
		}
		break;

	case NAV_CMD_LOITER_UNLIMITED:
	case NAV_CMD_LOITER_TIME_LIMIT: {
			if (loiter_achieved(item.lat, item.lon, altitude_amsl)) {
				waypoint_position_reached = true;
			}
		}
		break;

	case NAV_CMD_LOITER_TO_ALT: {
			// NAV_CMD_LOITER_TO_ALT only uses mission item altitude once it's in the loiter
			position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			//  first check if the altitude setpoint is the mission setpoint
			if (fabsf(curr_sp->alt - altitude_amsl) >= FLT_EPSILON) {
				if (loiter_achieved(item.lat, item.lon, curr_sp->alt)) {
					// now set the loiter to the final altitude in the NAV_CMD_LOITER_TO_ALT mission item
					curr_sp->alt = altitude_amsl;
					_navigator->set_position_setpoint_triplet_updated();
				}

			} else {
				if (loiter_achieved(item.lat, item.lon, altitude_amsl)) {
					waypoint_position_reached = true;
				}
			}
		}
		break;

	case NAV_CMD_DELAY:
		waypoint_position_reached = true;
		break;

	case NAV_CMD_WAYPOINT: {

			/* for vtol back transition calculate acceptance radius based on time and ground speed */
			// TODO: move this out of navigator and into VTOL
			if (_navigator->get_vstatus()->is_vtol && item.vtol_back_transition) {

				const float vx = _navigator->get_local_position()->vx;
				const float vy = _navigator->get_local_position()->vy;

				const float v = sqrtf(vx * vx + vy * vy);

				const float back_trans_dec = _navigator->get_vtol_back_trans_deceleration();
				const float reverse_delay = _navigator->get_vtol_reverse_delay();

				if (back_trans_dec > FLT_EPSILON && v > FLT_EPSILON) {
					mission_acceptance_radius = ((v / back_trans_dec / 2) * v) + reverse_delay * v;
				}
			}

			if (dist >= 0.0f &&
			    dist_xy <= mission_acceptance_radius &&
			    dist_z <= altitude_acceptance_radius) {

				waypoint_position_reached = true;
			}
		}
	}

	if (waypoint_position_reached) {
		if (!_waypoint_position_reached) {
			// reached just now
			_time_wp_reached = hrt_absolute_time();
		}

	} else {
		// reset
		_time_wp_reached = UINT64_MAX;
	}

	_waypoint_position_reached = waypoint_position_reached;

	return waypoint_position_reached;
}

bool
MissionBlock::loiter_achieved(const double &lat, const double &lon, const float &alt) const
{
	bool success = false;

	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	float dist_xy = -1.0f;
	float dist_z = -1.0f;
	const float dist = get_distance_to_point_global_wgs84(lat, lon, alt,
			   gpos.lat, gpos.lon, gpos.alt, &dist_xy, &dist_z);

	if (_navigator->get_vstatus()->is_rotary_wing) {
		if (dist >= 0.0f &&
		    dist_xy <= _navigator->get_acceptance_radius() &&
		    dist_z <= _navigator->get_altitude_acceptance_radius()) {

			success = true;
		}

	} else {

		/* Loiter mission item on a non rotary wing: the aircraft is going to circle the
		 * coordinates with a radius equal to the loiter_radius field. It is not flying
		 * through the waypoint center.
		 * Therefore the item is marked as reached once the system reaches the loiter
		 * radius (+ some margin). Time inside and turn count is handled elsewhere.
		 */
		if (dist >= 0.0f &&
		    dist_xy <= _navigator->get_acceptance_radius(fabsf(_mission_item.loiter_radius) * 1.2f) &&
		    dist_z <= _navigator->get_altitude_acceptance_radius()) {

			success = true;
		}
	}

	return success;
}

bool
MissionBlock::yaw_achieved(const mission_item_s &item) const
{
	// return true if there's no yaw requirement
	bool success = true;

	if (PX4_ISFINITE(item.yaw)) {

		bool yaw_required = false;

		if (_navigator->get_vstatus()->is_rotary_wing) {
			// multicopter yaw setpoint for waypoint and loiter
			if (item.nav_cmd == NAV_CMD_WAYPOINT ||
			    item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			    item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT) {

				yaw_required = true;

				const float yaw_err = wrap_pi(item.yaw - _navigator->get_local_position()->yaw);

				if (fabsf(yaw_err) > math::radians(_navigator->get_yaw_threshold())) {
					success = false;
				}
			}

		} else {
			// fixed wing required heading
			if (item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {

				const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

				if (item.force_heading && next_sp.valid) {

					yaw_required = true;

					const vehicle_global_position_s &gpos = *_navigator->get_global_position();
					const vehicle_local_position_s &lpos = *_navigator->get_local_position();

					const float yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, next_sp.lat, next_sp.lon);

					// check course
					const float cog = atan2f(lpos.vy, lpos.vx);
					const float yaw_err = wrap_pi(yaw - cog);

					if (fabsf(yaw_err) > math::radians(_navigator->get_yaw_threshold())) {
						success = false;
					}
				}
			}
		}

		if (yaw_required) {

			// check yaw timeout
			const float time_elapsed = hrt_elapsed_time(&_time_wp_reached) / 1e6f;
			const float yaw_timeout = _navigator->get_yaw_timeout();

			if (yaw_timeout > FLT_EPSILON && time_elapsed >= yaw_timeout) {

				if (!_navigator->get_mission_result()->failure) {

					_navigator->get_mission_result()->failure = true;
					_navigator->set_mission_result_updated();

					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "unable to reach heading within timeout");
				}

				success = false;
			}
		}
	}

	return success;
}

bool
MissionBlock::time_inside_finished(const mission_item_s &item) const
{
	bool success = false;
	const float time_inside = get_time_inside(item);

	if (time_inside > 0.0f) {
		const bool time_reached_valid = (_time_wp_reached != UINT64_MAX);
		const float time_elapsed = hrt_elapsed_time(&_time_wp_reached) / 1e6f;

		if (time_reached_valid && (time_elapsed > time_inside)) {
			success = true;
		}

	} else {
		// no time limit set
		success = true;
	}

	return success;
}

void
MissionBlock::set_loiter_exit(const mission_item_s &item)
{
	// set loiter exit for FW with a loiter mission item
	if (!_navigator->get_vstatus()->is_rotary_wing && item.loiter_exit_xtrack) {

		position_setpoint_s &curr_sp = _navigator->get_position_setpoint_triplet()->current;
		const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

		const bool curr_sp_loiter = (curr_sp.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		// exit xtrack location
		// reset lat/lon of loiter waypoint so vehicle follows a tangent
		if (curr_sp_loiter && curr_sp.valid && next_sp.valid) {

			const float range = get_distance_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

			if (PX4_ISFINITE(range) && range > 0.0f) {
				const float inner_angle = M_PI_2_F - asinf(item.loiter_radius / range);

				// Compute "ideal" tangent origin
				float bearing = get_bearing_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

				if (curr_sp.loiter_direction > 0) {
					bearing -= inner_angle;

				} else {
					bearing += inner_angle;
				}

				// Replace current setpoint lat/lon with tangent coordinate
				waypoint_from_heading_and_distance(curr_sp.lat, curr_sp.lon, bearing, curr_sp.loiter_radius,
								   &curr_sp.lat, &curr_sp.lon);
			}
		}
	}
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_time_wp_reached = 0;
}

void
MissionBlock::issue_command(const mission_item_s &item)
{
	if (item_contains_position(item)) {
		return;
	}

	// NAV_CMD_DO_LAND_START is only a marker
	if (item.nav_cmd == NAV_CMD_DO_LAND_START) {
		return;
	}

	if (item.nav_cmd == NAV_CMD_DO_SET_SERVO) {
		PX4_INFO("DO_SET_SERVO command");

		// XXX: we should issue a vehicle command and handle this somewhere else
		actuator_controls_s actuators = {};
		actuators.timestamp = hrt_absolute_time();

		// params[0] actuator number to be set 0..5 (corresponds to AUX outputs 1..6)
		// params[1] new value for selected actuator in ms 900...2000
		actuators.control[(int)item.params[0]] = 1.0f / 2000 * -item.params[1];

		if (_actuator_pub != nullptr) {
			orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &actuators);

		} else {
			_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators);
		}

	} else {
		_action_start = hrt_absolute_time();

		// mission_item -> vehicle_command

		// we're expecting a mission command item here so assign the "raw" inputs to the command
		// (MAV_FRAME_MISSION mission item)
		vehicle_command_s vcmd = {};
		vcmd.command = item.nav_cmd;
		vcmd.param1 = item.params[0];
		vcmd.param2 = item.params[1];
		vcmd.param3 = item.params[2];
		vcmd.param4 = item.params[3];

		if (item.nav_cmd == NAV_CMD_DO_SET_ROI_LOCATION && item.altitude_is_relative) {
			vcmd.param5 = item.lat;
			vcmd.param6 = item.lon;
			vcmd.param7 = item.altitude + _navigator->get_home_position()->alt;

		} else {
			vcmd.param5 = (double)item.params[4];
			vcmd.param6 = (double)item.params[5];
			vcmd.param7 = item.params[6];
		}

		_navigator->publish_vehicle_cmd(&vcmd);
	}
}

float
MissionBlock::get_time_inside(const mission_item_s &item) const
{
	if ((item.nav_cmd == NAV_CMD_WAYPOINT && _navigator->get_vstatus()->is_rotary_wing) ||
	    item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	    item.nav_cmd == NAV_CMD_DELAY) {

		// TODO: set appropriate upper limit
		return math::constrain(item.time_inside, 0.0f, 3600.0f);
	}

	return 0.0f;
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
	       item.nav_cmd == NAV_CMD_VTOL_LAND ||
	       item.nav_cmd == NAV_CMD_DO_FOLLOW_REPOSITION;
}

bool
MissionBlock::mission_item_to_position_setpoint(const mission_item_s &item, position_setpoint_s *sp)
{
	/* don't change the setpoint for non-position items */
	if (!item_contains_position(item)) {
		return false;
	}

	sp->lat = item.lat;
	sp->lon = item.lon;
	sp->alt = get_absolute_altitude_for_item(item);
	sp->yaw = item.yaw;
	sp->yaw_valid = PX4_ISFINITE(item.yaw);
	sp->loiter_radius = (fabsf(item.loiter_radius) > NAV_EPSILON_POSITION) ? fabsf(item.loiter_radius) :
			    _navigator->get_loiter_radius();
	sp->loiter_direction = (item.loiter_radius > 0) ? 1 : -1;
	sp->acceptance_radius = item.acceptance_radius;

	sp->cruising_speed = _navigator->get_cruising_speed();
	sp->cruising_throttle = _navigator->get_cruising_throttle();

	switch (item.nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:

		// if already flying (armed and !landed) treat TAKEOFF like regular POSITION
		if ((_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !_navigator->get_land_detected()->landed && !_navigator->get_land_detected()->maybe_landed) {

			sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		} else {
			sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

			// set pitch and ensure that the hold time is zero
			sp->pitch_min = item.pitch_min;
		}

		break;

	case NAV_CMD_VTOL_TAKEOFF:
		sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		break;

	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LAND;
		break;

	case NAV_CMD_LOITER_TO_ALT:

		// initially use current altitude, and switch to mission item altitude once in loiter position
		if (_navigator->get_loiter_min_alt() > 0.0f) { // ignore _param_loiter_min_alt if smaller then 0 (-1)
			sp->alt = math::max(_navigator->get_global_position()->alt,
					    _navigator->get_home_position()->alt + _navigator->get_loiter_min_alt());

		} else {
			sp->alt = _navigator->get_global_position()->alt;
		}

	// fall through
	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_UNLIMITED:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		break;

	default:
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;
	}

	sp->valid = true;

	return sp->valid;
}

void
MissionBlock::set_loiter_item(struct mission_item_s *item, float min_clearance)
{
	if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {
		item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		if (_navigator->get_can_loiter_at_sp() && pos_sp_triplet->current.valid) {
			/* use current position setpoint */
			item->lat = pos_sp_triplet->current.lat;
			item->lon = pos_sp_triplet->current.lon;
			item->altitude = pos_sp_triplet->current.alt;

		} else {
			/* use current position and use return altitude as clearance */
			item->lat = _navigator->get_global_position()->lat;
			item->lon = _navigator->get_global_position()->lon;
			item->altitude = _navigator->get_global_position()->alt;

			if (min_clearance > 0.0f && item->altitude < _navigator->get_home_position()->alt + min_clearance) {
				item->altitude = _navigator->get_home_position()->alt + min_clearance;
			}
		}

		item->altitude_is_relative = false;
		item->yaw = NAN;
		item->loiter_radius = _navigator->get_loiter_radius();
		item->acceptance_radius = _navigator->get_acceptance_radius();
		item->time_inside = 0.0f;
		item->autocontinue = false;
		item->origin = ORIGIN_ONBOARD;
	}
}

void
MissionBlock::set_takeoff_item(struct mission_item_s *item, float abs_altitude, float min_pitch)
{
	item->nav_cmd = NAV_CMD_TAKEOFF;

	/* use current position */
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->yaw = _navigator->get_global_position()->yaw;

	item->altitude = abs_altitude;
	item->altitude_is_relative = false;

	item->loiter_radius = _navigator->get_loiter_radius();
	item->pitch_min = min_pitch;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

void
MissionBlock::set_land_item(struct mission_item_s *item, bool at_current_location)
{
	/* VTOL transition to RW before landing */
	if (_navigator->force_vtol()) {

		vehicle_command_s vcmd = {};
		vcmd.command = NAV_CMD_DO_VTOL_TRANSITION;
		vcmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		_navigator->publish_vehicle_cmd(&vcmd);
	}

	/* set the land item */
	item->nav_cmd = NAV_CMD_LAND;

	/* use current position */
	if (at_current_location) {
		item->lat = (double)NAN; //descend at current position
		item->lon = (double)NAN; //descend at current position
		item->yaw = _navigator->get_local_position()->yaw;

	} else {
		/* use home position */
		item->lat = _navigator->get_home_position()->lat;
		item->lon = _navigator->get_home_position()->lon;
		item->yaw = _navigator->get_home_position()->yaw;
	}

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
	item->yaw = _navigator->get_global_position()->yaw;
	item->autocontinue = true;
}

void
MissionBlock::mission_apply_limitation(mission_item_s &item)
{
	/*
	 * Limit altitude
	 */

	/* do nothing if altitude max is negative */
	if (_navigator->get_land_detected()->alt_max > 0.0f) {

		/* absolute altitude */
		float altitude_abs = item.altitude_is_relative
				     ? item.altitude + _navigator->get_home_position()->alt
				     : item.altitude;

		/* limit altitude to maximum allowed altitude */
		if ((_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt) < altitude_abs) {
			item.altitude = item.altitude_is_relative ?
					_navigator->get_land_detected()->alt_max :
					_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt;

		}
	}

	/*
	 * Add other limitations here
	 */
}

float
MissionBlock::get_absolute_altitude_for_item(const mission_item_s &mission_item) const
{
	if (mission_item.altitude_is_relative) {
		return mission_item.altitude + _navigator->get_home_position()->alt;

	} else {
		return mission_item.altitude;
	}
}
