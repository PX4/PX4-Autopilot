/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.cpp
 * Helper class to access RTL
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl.h"
#include "navigator.h"

#include <dataman/dataman.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <navigator/navigation.h>

#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/uORB.h>

static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_rtl_state(RTL_STATE_NONE),
	_param_return_alt(this, "RTL_RETURN_ALT", false),
	_param_min_loiter_alt(this, "MIS_LTRMIN_ALT", false),
	_param_descend_alt(this, "RTL_DESCEND_ALT", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false),
	_param_rtl_min_dist(this, "RTL_MIN_DIST", false)
{
	/* load initial params */
	updateParams();

	/* initial reset */
	on_inactive();
}

void
RTL::on_inactive()
{
	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

float
RTL::get_rtl_altitude()
{
	return math::min(_param_return_alt.get(), _navigator->get_land_detected()->alt_max);
}

void
RTL::on_activation()
{
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	// find the RTL destination: go through the safe points & home position
	const home_position_s &home_position = *_navigator->get_home_position();
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	int closest_index = 0;
	mission_safe_point_s closest_safe_point;

	// take home position into account
	double dlat = home_position.lat - global_position.lat;
	double dlon = home_position.lon - global_position.lon;
	double min_dist_squared = dlat * dlat + dlon * dlon;

	// find the closest point
	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		mission_safe_point_s mission_safe_point;

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_safe_point, sizeof(mission_safe_point_s)) !=
		    sizeof(mission_safe_point_s)) {
			PX4_ERR("dm_read failed");
			continue;
		}

		// TODO: handle mission_safe_point.frame
		// TODO: take altitude into account for distance measurement

		dlat = mission_safe_point.lat - global_position.lat;
		dlon = mission_safe_point.lon - global_position.lon;
		double dist_squared = dlat * dlat + dlon * dlon;

		if (dist_squared < min_dist_squared) {
			closest_index = current_seq;
			min_dist_squared = dist_squared;
			closest_safe_point = mission_safe_point;
		}
	}



	if (closest_index == 0) {
		_destination.set(home_position);

	} else {
		_destination.safe_point_index = closest_index;
		_destination.lat = closest_safe_point.lat;
		_destination.lon = closest_safe_point.lon;

		// TODO: for now we use the same altitude as home
		_destination.alt = home_position.alt;
		_destination.yaw = home_position.yaw;
	}

	/* for safety reasons don't go into RTL if landed */
	if (_navigator->get_land_detected()->landed) {
		_rtl_state = RTL_STATE_LANDED;

	} else if (global_position.alt < _destination.alt + get_rtl_altitude()) {

		/* if lower than return altitude, climb up first */
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		/* otherwise go straight to return */
		_rtl_state = RTL_STATE_RETURN;

		/* set altitude setpoint to current altitude */
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = global_position.alt;
	}

	set_rtl_item();
}

void
RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}
}

void
RTL::set_rtl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	_navigator->set_can_loiter_at_sp(false);

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			// check if we are pretty close to destination already
			float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
						 _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

			// if we are close to the destination we do not climb as high, otherwise we climb to return alt
			float climb_alt = _destination.alt + get_rtl_altitude();

			// we are close to the destination, limit climb to min
			if (destination_dist < _param_rtl_min_dist.get()) {
				climb_alt = _destination.alt + _param_descend_alt.get();
			}

			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = climb_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above destination)",
						     (int)(climb_alt),
						     (int)(climb_alt - _destination.alt));
			break;
		}

	case RTL_STATE_RETURN: {
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			// don't change altitude

			// use home yaw if close to the destination
			/* check if we are pretty close to the destination already */
			float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
						 _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

			if (destination_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = _destination.yaw;

			} else {
				// use current heading to destination
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
							    _destination.lat, _destination.lon);
			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above destination)",
						     (int)(_mission_item.altitude),
						     (int)(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			_mission_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_mission_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _destination.alt + _param_descend_alt.get();

			// check if we are already lower - then we will just stay there
			if (_mission_item.altitude > _navigator->get_global_position()->alt) {
				_mission_item.altitude = _navigator->get_global_position()->alt;
			}

			_mission_item.yaw = _destination.yaw;

			// except for vtol which might be still off here and should point towards this location
			float d_current = get_distance_to_next_waypoint(
						  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
						  _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && d_current > _navigator->get_acceptance_radius()) {
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
							    _mission_item.lat, _mission_item.lon);
			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above destination)",
						     (int)(_mission_item.altitude),
						     (int)(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			// don't change altitude
			_mission_item.yaw = _destination.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = autoland ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loiter");
			}

			break;
		}

	case RTL_STATE_LAND: {
			set_land_item(&_mission_item, false);
			_mission_item.yaw = _destination.yaw;

			if (_destination.safe_point_index == 0) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at home");

			} else {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at safe point %i",
							     _destination.safe_point_index);
			}

			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* execute command if set. This is required for commands like VTOL transition */
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;

		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_DESCEND:

		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}
