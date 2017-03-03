/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

#include <cfloat>

#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/uORB.h>

using math::max;
using math::min;

#define DELAY_SIGMA	0.01f

RTL::RTL(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_return_alt(this, "RTL_RETURN_ALT", false),
	_param_min_loiter_alt(this, "MIS_LTRMIN_ALT", false),
	_param_descend_alt(this, "RTL_DESCEND_ALT", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false),
	_param_rtl_min_dist(this, "RTL_MIN_DIST", false),
	_param_rtl_land_type(this, "RTL_LAND_TYPE", false)
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
	return min(_param_return_alt.get(), _navigator->get_land_detected()->alt_max);
}

void
RTL::on_activation()
{
	/* reset starting point so we override what the triplet contained from the previous navigation state */
	_rtl_start_lock = false;
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

	/* for safety reasons don't go into RTL if landed */
	if (_navigator->get_land_detected()->landed) {
		_rtl_state = RTL_STATE_LANDED;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Already landed, not executing RTL");

	} else if (_navigator->get_global_position()->alt < (_navigator->get_home_position()->alt + get_rtl_altitude())) {
		/* if lower than return altitude, climb up first */

		_rtl_state = RTL_STATE_CLIMB;

	} else {
		/* otherwise go straight to return */

		/* set altitude setpoint to current altitude */
		_rtl_state = RTL_STATE_RETURN;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_global_position()->alt;
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
	if (!_rtl_start_lock) {
		set_previous_pos_setpoint();
	}

	_navigator->set_can_loiter_at_sp(false);

	const struct home_position_s &home = *_navigator->get_home_position();
	const struct vehicle_global_position_s &gpos = *_navigator->get_global_position();

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			// check if we are pretty close to home already
			float home_dist = get_distance_to_next_waypoint(home.lat, home.lon, gpos.lat, gpos.lon);

			// if we are close to home we do not climb as high, otherwise we climb to return alt
			float climb_alt = home.alt + get_rtl_altitude();

			// we are close to home, limit climb to min
			if (home_dist < _param_rtl_min_dist.get()) {
				climb_alt = home.alt + _param_min_loiter_alt.get();
			}

			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = climb_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home)",
					 (int)(climb_alt),
					 (int)(climb_alt - _navigator->get_home_position()->alt));
			break;
		}

	case RTL_STATE_RETURN: {

			if (_param_rtl_land_type.get() == 1) {
				// landing using planned mission landing, fly to DO_LAND_START instead of returning HOME
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				_mission_item.nav_cmd = NAV_CMD_DO_LAND_START;

			} else {
				_mission_item.lat = home.lat;
				_mission_item.lon = home.lon;
				// don't change altitude

				// use home yaw if close to home
				/* check if we are pretty close to home already */
				float home_dist = get_distance_to_next_waypoint(home.lat, home.lon, gpos.lat, gpos.lon);

				if (home_dist < _param_rtl_min_dist.get()) {
					_mission_item.yaw = home.yaw;

				} else {
					if (pos_sp_triplet->previous.valid) {
						/* if previous setpoint is valid then use it to calculate heading to home */
						_mission_item.yaw = get_bearing_to_next_waypoint(pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon,
								    _mission_item.lat, _mission_item.lon);

					} else {
						/* else use current position */
						_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);
					}
				}

				_mission_item.loiter_radius = _navigator->get_loiter_radius();
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
				_mission_item.time_inside = 0.0f;
				_mission_item.autocontinue = true;
				_mission_item.origin = ORIGIN_ONBOARD;

				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above home)",
						 (int)(_mission_item.altitude),
						 (int)(_mission_item.altitude - home.alt));

				_rtl_start_lock = true;
			}

			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			_mission_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_mission_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = home.alt + _param_descend_alt.get();

			// check if we are already lower - then we will just stay there
			if (_mission_item.altitude > gpos.alt) {
				_mission_item.altitude = gpos.alt;
			}

			_mission_item.yaw = home.yaw;

			// except for vtol which might be still off here and should point towards this location
			float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && d_current > _navigator->get_acceptance_radius()) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);
			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = false;
			_mission_item.origin = ORIGIN_ONBOARD;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above home)",
					 (int)(_mission_item.altitude),
					 (int)(_mission_item.altitude - home.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			// don't change altitude
			_mission_item.yaw = home.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = max(_param_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland && (Navigator::get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
						 (double)Navigator::get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loiter");
			}

			break;
		}

	case RTL_STATE_LAND: {

			// land at home position
			_mission_item.yaw = home.yaw;
			set_land_item(&_mission_item, false);

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at home");

			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);

			mavlink_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, landed");
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	/* execute command if set */
	issue_command(&_mission_item);

	/* convert mission item to current position setpoint and make it valid */
	if (mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current)) {
		pos_sp_triplet->next.valid = false;
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:

		if (_param_rtl_land_type.get() == 1) {
			// DO_LAND_START planned mission landing
			// don't advance past RETURN

		} else if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;

		} else {
			_rtl_state = RTL_STATE_DESCEND;
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
